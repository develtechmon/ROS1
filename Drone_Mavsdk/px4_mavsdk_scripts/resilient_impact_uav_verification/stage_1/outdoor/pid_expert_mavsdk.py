import threading
"""
PID EXPERT CONTROLLER FOR HOVER - MAVSDK/PX4 VERSION
=====================================================
Direct translation of pid_expert_v2.py from AirSim to MAVSDK.

KEY DIFFERENCES FROM AIRSIM VERSION:
  - State is collected from async telemetry streams (not a single sync call)
  - Actions are sent via offboard.set_velocity_ned() (not moveByVelocityAsync)
  - Coordinate frame: NED throughout (North=+x, East=+y, Down=+z)
    → Altitude target is NEGATIVE in NED (e.g. hover at 10m = z_ned = -10.0)
  - MAVSDK auto-resends offboard setpoints at 20Hz internally
    → We still call set_velocity_ned() at our control rate (20Hz) to UPDATE the setpoint
  - No client.reset() — episode management is handled separately

THINK OF IT THIS WAY:
  AirSim state = one synchronous snapshot (like a photograph)
  MAVSDK state = a live dashboard with separate dials being updated async
  We read all dials into a shared buffer, then snapshot them together.

Usage (SITL):
    Terminal 1: make px4_sitl gazebo
    Terminal 2: python pid_expert_mavsdk.py

For HITL run this:
python pid_expert_mavsdk.py --address serial:///dev/ttyACM0:921600

or you run run directly without address serial because we'll use the proxy
python pid_expert_mavsdk.py 

Expected Output:
    Mean altitude: ~10.0m
    Std deviation: <0.15m
    Max error: <0.5m
    Status: EXCELLENT! Ready to collect demonstrations.
"""

import asyncio
import numpy as np
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw


# ─────────────────────────────────────────────
# Shared Telemetry Buffer
# ─────────────────────────────────────────────
class TelemetryBuffer:
    """
    Collects async telemetry streams into a single readable snapshot.

    ANALOGY: Think of this as a car dashboard.
    Each gauge (speed, rpm, fuel) updates independently.
    When you glance at the dashboard, you read all gauges at once.
    This class IS that dashboard — async streams update it continuously,
    your control loop reads it at 20Hz.
    """

    def __init__(self):
        # Position (NED, meters) — origin = arming point
        self.pos_n = 0.0   # North
        self.pos_e = 0.0   # East
        self.pos_d = 0.0   # Down (negative = above ground)

        # Velocity (NED, m/s)
        self.vel_n = 0.0
        self.vel_e = 0.0
        self.vel_d = 0.0

        # Orientation (Euler, radians)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Quaternion (for obs vector, matches AirSim order: w, x, y, z)
        self.q_w = 1.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0

        # Angular velocity (body frame, rad/s)
        self.ang_vel_x = 0.0  # roll rate
        self.ang_vel_y = 0.0  # pitch rate
        self.ang_vel_z = 0.0  # yaw rate

        self.ready = False  # True once first telemetry received

    def get_obs(self):
        """
        Returns 13-dim observation vector — identical structure to AirSim version:
        [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d,
         q_w, q_x, q_y, q_z, ang_vel_x, ang_vel_y, ang_vel_z]

        NOTE on coordinate mapping from AirSim:
          AirSim x = MAVSDK North (pos_n)
          AirSim y = MAVSDK East  (pos_e)
          AirSim z = MAVSDK Down  (pos_d)   ← both NED, same sign
        """
        return np.array([
            self.pos_n, self.pos_e, self.pos_d,
            self.vel_n, self.vel_e, self.vel_d,
            self.q_w,   self.q_x,   self.q_y,   self.q_z,
            self.ang_vel_x, self.ang_vel_y, self.ang_vel_z
        ], dtype=np.float32)

    def get_state_dict(self):
        """Returns state dict compatible with PIDExpertHover.get_action()"""
        return {
            'position':         np.array([self.pos_n, self.pos_e, self.pos_d]),
            'velocity':         np.array([self.vel_n, self.vel_e, self.vel_d]),
            'orientation':      np.array([self.q_w, self.q_x, self.q_y, self.q_z]),
            'angular_velocity': np.array([self.ang_vel_x, self.ang_vel_y, self.ang_vel_z])
        }

    def altitude(self):
        """Returns altitude in meters (positive = above ground)"""
        return -self.pos_d  # NED: down is positive, so negate for altitude


# ─────────────────────────────────────────────
# Telemetry Subscriber Tasks
# ─────────────────────────────────────────────
async def subscribe_position(drone, buf):
    """Continuously updates buf with latest NED position"""
    async for pos in drone.telemetry.position_velocity_ned():
        buf.pos_n = pos.position.north_m
        buf.pos_e = pos.position.east_m
        buf.pos_d = pos.position.down_m
        buf.vel_n = pos.velocity.north_m_s
        buf.vel_e = pos.velocity.east_m_s
        buf.vel_d = pos.velocity.down_m_s
        buf.ready = True


async def subscribe_attitude(drone, buf):
    """Continuously updates buf with latest attitude (quaternion + euler)"""
    async for att in drone.telemetry.attitude_quaternion():
        buf.q_w = att.w
        buf.q_x = att.x
        buf.q_y = att.y
        buf.q_z = att.z


async def subscribe_angular_velocity(drone, buf):
    """Continuously updates buf with latest body-frame angular velocity"""
    async for ang_vel in drone.telemetry.attitude_angular_velocity_body():
        buf.ang_vel_x = ang_vel.roll_rad_s
        buf.ang_vel_y = ang_vel.pitch_rad_s
        buf.ang_vel_z = ang_vel.yaw_rad_s

# ─────────────────────────────────────────────
# Main Test
# ─────────────────────────────────────────────
async def goto_position_ned(drone, buf, north_m, east_m, alt_m,
                            speed_m_s=5.0, tolerance_m=0.3, timeout_s=30.0):
    """
    Fly to a NED position using offboard position setpoints and wait for arrival.

    This is the MAVSDK equivalent of AirSim's moveToPositionAsync().
    Offboard must already be started before calling this.

    Args:
        north_m, east_m: horizontal target (metres from arm point)
        alt_m:           target altitude (positive = above ground)
        speed_m_s:       not directly settable via position NED, but the
                         PX4 MPC_XY_VEL_MAX / MPC_Z_VEL_MAX params govern climb
        tolerance_m:     3D distance threshold to declare arrival
        timeout_s:       give up after this long
    """
    from mavsdk.offboard import PositionNedYaw
    target_down = -alt_m   # NED: altitude → negative down

    # Send position setpoint continuously until arrival
    # (MAVSDK requires setpoints to keep flowing at ≥2Hz in offboard mode)
    t_start = asyncio.get_event_loop().time()
    while True:
        await drone.offboard.set_position_ned(
            PositionNedYaw(north_m, east_m, target_down, 0.0)
        )

        # 3D distance to target
        dn = buf.pos_n - north_m
        de = buf.pos_e - east_m
        dd = buf.pos_d - target_down
        dist_3d = np.sqrt(dn**2 + de**2 + dd**2)

        print(f"  Climbing... Alt={buf.altitude():.2f}m  3D-err={dist_3d:.2f}m",
              end="\r", flush=True)

        if dist_3d <= tolerance_m:
            print(f"\n  Arrived at ({north_m:.1f}, {east_m:.1f}, {alt_m:.1f}m) "
                  f"— 3D error={dist_3d:.2f}m")
            break

        if asyncio.get_event_loop().time() - t_start > timeout_s:
            print(f"\n  Timeout. Alt={buf.altitude():.2f}m, 3D-err={dist_3d:.2f}m — proceeding")
            break

        await asyncio.sleep(0.05)   # 20Hz setpoint rate


async def test_position_hold_quality(system_address="udp://:14550"):
    """
    Verifies that PX4's position controller holds altitude well enough
    to produce quality training data.

    WHAT THIS TESTS (new approach):
      - PX4 position hold quality at 10m (this IS the expert now)
      - Computed action label distribution (should be near zero at hover)
      - State distribution quality (low variance = clean training data)

    WHAT WE NO LONGER TEST:
      - Our own PID gains (irrelevant — we removed our PID from data collection)
      - Velocity mode stability (irrelevant — collection stays in position mode)

    PASS CRITERIA:
      Altitude std  < 0.15m  → PX4 position hold is clean enough
      Mean alt      > 9.8m   → Close to target
      Mean |action| < 0.3    → Labels are small corrections, not large commands
    """
    print("\n" + "="*70)
    print("TESTING PX4 POSITION HOLD QUALITY FOR DATA COLLECTION")
    print("(This verifies the expert — PX4 itself — not our PID)")
    print("="*70)

    drone = System()
    await drone.connect(system_address=system_address)

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("  Connected!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("  Global position OK")
            break

    await drone.telemetry.set_rate_position_velocity_ned(20.0)
    await drone.telemetry.set_rate_attitude(20.0)

    buf = TelemetryBuffer()
    asyncio.ensure_future(subscribe_position(drone, buf))
    asyncio.ensure_future(subscribe_attitude(drone, buf))
    asyncio.ensure_future(subscribe_angular_velocity(drone, buf))

    print("Waiting for telemetry buffer...")
    while not buf.ready:
        await asyncio.sleep(0.1)
    print("  Telemetry ready!")

    # Arm
    print("Arming...")
    await drone.action.arm()
    await asyncio.sleep(1.0)

    # Enter offboard and climb to 10m via position setpoint
    from mavsdk.offboard import PositionNedYaw
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"  Offboard start failed: {e}")
        await drone.action.disarm()
        return

    print("Moving to 10m via PX4 position control...")
    await goto_position_ned(drone, buf,
                            north_m=0.0, east_m=0.0, alt_m=10.0,
                            tolerance_m=0.3, timeout_s=40.0)

    # Settle
    print("Settling at 10m...")
    t_settle = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - t_settle < 2.0:
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))
        await asyncio.sleep(0.05)

    # ── Run position hold quality test ──
    # Same as what data collection does: stay in position mode, compute labels
    TARGET_N, TARGET_E, TARGET_D = 0.0, 0.0, -10.0
    KP_N, KP_E, KP_D = 0.5, 0.5, 0.8
    V_MAX = 3.0

    print(f"\nRunning position hold quality test — 100 steps (5 seconds)...\n")
    print(f"{'Step':>5}  {'Alt (m)':>8}  {'Dist (m)':>9}  "
          f"{'vn_lbl':>8}  {'ve_lbl':>8}  {'vd_lbl':>8}")
    print("-" * 62)

    positions = []
    actions   = []

    for step in range(100):
        # Keep sending position setpoint — PX4 holds the drone
        await drone.offboard.set_position_ned(
            PositionNedYaw(TARGET_N, TARGET_E, TARGET_D, 0.0)
        )

        # Compute what the action label would be at this state
        err_n  = TARGET_N - buf.pos_n
        err_e  = TARGET_E - buf.pos_e
        err_d  = TARGET_D - buf.pos_d
        vn_lbl = float(np.clip(KP_N * err_n, -V_MAX, V_MAX))
        ve_lbl = float(np.clip(KP_E * err_e, -V_MAX, V_MAX))
        vd_lbl = float(np.clip(KP_D * err_d, -V_MAX, V_MAX))

        alt  = buf.altitude()
        dist = np.sqrt(buf.pos_n**2 + buf.pos_e**2)
        positions.append(alt)
        actions.append([vn_lbl, ve_lbl, vd_lbl])

        if step % 20 == 0:
            print(f"{step:>5}  {alt:>8.3f}  {dist:>9.3f}  "
                  f"{vn_lbl:>8.3f}  {ve_lbl:>8.3f}  {vd_lbl:>8.3f}")

        await asyncio.sleep(0.05)

    # Stop and land
    await drone.offboard.stop()
    await asyncio.sleep(0.5)
    await drone.action.land()

    # Statistics
    positions = np.array(positions)
    actions   = np.array(actions)

    mean_alt   = np.mean(positions)
    std_alt    = np.std(positions)
    max_error  = np.max(np.abs(positions - 10.0))
    mean_action_mag = np.mean(np.abs(actions))

    print("\n" + "="*70)
    print("RESULTS")
    print("="*70)
    print(f"PX4 position hold quality:")
    print(f"  Mean altitude   : {mean_alt:.3f}m  (target: 10.0m)")
    print(f"  Std deviation   : {std_alt:.3f}m")
    print(f"  Max error       : {max_error:.3f}m")
    print(f"\nAction label quality:")
    print(f"  Mean |action|   : {mean_action_mag:.4f} m/s")
    print(f"  Mean |vn label| : {np.mean(np.abs(actions[:,0])):.4f} m/s")
    print(f"  Mean |ve label| : {np.mean(np.abs(actions[:,1])):.4f} m/s")
    print(f"  Mean |vd label| : {np.mean(np.abs(actions[:,2])):.4f} m/s")

    print()
    pos_ok    = std_alt < 0.15 and mean_alt > 9.8
    action_ok = mean_action_mag < 0.3
    if pos_ok and action_ok:
        print("  EXCELLENT — PX4 position hold is clean.")
        print("  Action labels are small corrections near zero.")
        print("  Ready to collect demonstrations.")
    elif pos_ok:
        print("  GOOD — Position hold OK, but action labels are large.")
        print("  Check: is the drone starting far from target?")
    else:
        print("  POOR — PX4 position hold is unstable.")
        print("  Check: EKF2 health, GPS quality, wind in SITL.")
    print("="*70 + "\n")


if __name__ == "__main__":
    # Suppress the gRPC/asyncio event-loop-closed traceback that mavsdk 1.4.x
    # throws on shutdown when background telemetry streams are still running.
    # This is a known cleanup race in this version — it does not affect results.
    # Simple, clean execution.
    # We use asyncio.run() and let os._exit(0) terminate the process cleanly.
    # This avoids the gRPC/asyncio "Event loop is closed" traceback that appears
    # in mavsdk 1.4.x on Python 3.8 when background telemetry streams are still
    # running at shutdown. The root cause: concurrent.futures catches the gRPC
    # callback exception and prints it directly to stderr before threading.excepthook
    # can intercept it. os._exit(0) terminates all threads immediately — the gRPC
    # callbacks never fire into a closed loop.
    import os
    import argparse
    parser = argparse.ArgumentParser(description="PX4 position hold quality test")
    parser.add_argument(
        '--address', type=str, default='udp://:14550',
        help='SITL: udp://:14550   HITL: serial:///dev/ttyACM0:57600'
    )
    args = parser.parse_args()
    asyncio.run(test_position_hold_quality(system_address=args.address))
    os._exit(0)
