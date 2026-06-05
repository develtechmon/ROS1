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
# PID Controller (unchanged from AirSim version)
# ─────────────────────────────────────────────
class PIDController:
    """Simple PID controller — identical to AirSim version"""

    def __init__(self, kp, ki, kd, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral = 0
        self.previous_error = 0

    def update(self, error, dt):
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error
        output = p_term + i_term + d_term
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
        return output

    def reset(self):
        self.integral = 0
        self.previous_error = 0

    def preload(self, initial_error, desired_output=0.0):
        """
        Pre-load integral for bumpless transfer.
        Sets integral so output starts at desired_output for the given error,
        rather than building from zero over several seconds.
        """
        if self.ki != 0:
            self.integral = (desired_output - self.kp * initial_error) / self.ki
        self.previous_error = initial_error


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
# PID Expert (MAVSDK version)
# ─────────────────────────────────────────────
class PIDExpertHover:
    """
    Expert PID controller for hovering — MAVSDK version.

    Logic is IDENTICAL to AirSim pid_expert_v2.py.
    Only difference: get_action() receives state_dict from TelemetryBuffer,
    and actions are applied via offboard.set_velocity_ned().

    COORDINATE NOTE:
      - Target altitude 10m → pos_d target = -10.0 (NED, down positive)
      - error_z = target_z_ned - current_z_ned = -10.0 - pos_d
      - Positive vz_cmd = move DOWN in NED = descend
      - Negative vz_cmd = move UP in NED = ascend
      This matches AirSim NED convention exactly.
    """

    def __init__(self, target_altitude=10.0):
        self.target_altitude = target_altitude
        self.target_z_ned = -target_altitude  # NED: 10m up = -10m down
        self.dt = 0.05  # 20 Hz

        # Position PIDs
        # z gains tuned for MAVSDK/PX4 SITL — ki raised to close the ~0.8m
        # steady-state altitude error. Root cause: at ki=0.02, integral builds
        # only 0.08 m/s correction over 5s — not enough to hold against SITL drift
        # after switching from position to velocity setpoints.
        self.pid_x = PIDController(kp=0.5, ki=0.01, kd=0.3, output_limits=(-3, 3))
        self.pid_y = PIDController(kp=0.5, ki=0.01, kd=0.3, output_limits=(-3, 3))
        self.pid_z = PIDController(kp=1.0, ki=0.08, kd=0.5, output_limits=(-3, 3))

        # Velocity PIDs
        self.pid_vx = PIDController(kp=0.3, ki=0.01, kd=0.1,  output_limits=(-5, 5))
        self.pid_vy = PIDController(kp=0.3, ki=0.01, kd=0.1,  output_limits=(-5, 5))
        self.pid_vz = PIDController(kp=0.5, ki=0.05, kd=0.15, output_limits=(-5, 5))

        print("✓ PID Expert Controller Initialized (MAVSDK version, 13 observations)")
        print(f"  Target: Hover at {target_altitude}m (NED z = {self.target_z_ned}m)")
        print(f"  Control frequency: {1/self.dt:.0f} Hz")

    def get_action(self, state):
        """
        Given state dict, returns [vn_cmd, ve_cmd, vd_cmd] in NED m/s.

        NOTE: Return value maps to VelocityNedYaw(north, east, down, yaw_deg)
              vd_cmd positive = descend, negative = ascend.
        """
        pos = state['position']   # [north, east, down]
        vel = state['velocity']   # [vn, ve, vd]

        # Position errors in NED
        error_n = 0.0 - pos[0]                    # target north = 0
        error_e = 0.0 - pos[1]                    # target east = 0
        error_d = self.target_z_ned - pos[2]      # target down = -10.0

        # Position PIDs → desired velocities
        desired_vn = self.pid_x.update(error_n, self.dt)
        desired_ve = self.pid_y.update(error_e, self.dt)
        desired_vd = self.pid_z.update(error_d, self.dt)

        # Velocity errors
        vel_error_n = desired_vn - vel[0]
        vel_error_e = desired_ve - vel[1]
        vel_error_d = desired_vd - vel[2]

        # Velocity PIDs → commands
        vn_cmd = self.pid_vx.update(vel_error_n, self.dt)
        ve_cmd = self.pid_vy.update(vel_error_e, self.dt)
        vd_cmd = self.pid_vz.update(vel_error_d, self.dt)

        return np.array([vn_cmd, ve_cmd, vd_cmd], dtype=np.float32)

    def reset(self, state=None):
        """
        Reset all PIDs. If state provided, pre-load z integrals so the
        controller has immediate authority when switching from position
        to velocity setpoint mode — prevents altitude sink on transition.
        """
        for pid in [self.pid_x, self.pid_y, self.pid_z,
                    self.pid_vx, self.pid_vy, self.pid_vz]:
            pid.reset()
        if state is not None:
            pos = state['position']
            vel = state['velocity']
            # Pre-load position z: want desired_vd=0 at current error
            error_d = self.target_z_ned - pos[2]
            self.pid_z.preload(error_d, desired_output=0.0)
            # Pre-load velocity z: want vd_cmd=0 at current vel
            self.pid_vz.preload(0.0 - vel[2], desired_output=0.0)


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


async def test_pid_expert():
    print("\n" + "="*70)
    print("TESTING PID EXPERT CONTROLLER - MAVSDK/PX4 SITL")
    print("="*70)

    drone = System()
    await drone.connect(system_address="udp://:14550")

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

    # Set telemetry rates
    # NOTE: set_rate_attitude() covers quaternion, euler, AND angular velocity in mavsdk 1.4.x
    await drone.telemetry.set_rate_position_velocity_ned(20.0)
    await drone.telemetry.set_rate_attitude(20.0)

    # Start telemetry buffer
    buf = TelemetryBuffer()
    asyncio.ensure_future(subscribe_position(drone, buf))
    asyncio.ensure_future(subscribe_attitude(drone, buf))
    asyncio.ensure_future(subscribe_angular_velocity(drone, buf))

    print("Waiting for telemetry buffer to fill...")
    while not buf.ready:
        await asyncio.sleep(0.1)
    print("  Telemetry ready!")

    # ── Arm ──
    print("Arming...")
    await drone.action.arm()
    await asyncio.sleep(1.0)

    # ── Enter offboard with a position setpoint at ground level first ──
    # CRITICAL: Must set a setpoint BEFORE offboard.start() or it will be rejected.
    # We set it at current position (ground) so the drone doesn't jump.
    from mavsdk.offboard import PositionNedYaw
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, 0.0, 0.0)   # ground level, origin
    )
    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"  Offboard start failed: {e}")
        await drone.action.disarm()
        return

    # ── Climb to 10m via offboard position setpoint ──
    # This is the MAVSDK equivalent of AirSim's:
    #   client.moveToPositionAsync(0, 0, -10, 5).join()
    # PX4 position controller handles the climb smoothly using its own
    # velocity limits — no manual velocity ramping needed.
    print("Moving to 10m altitude via offboard position control...")
    await goto_position_ned(drone, buf,
                            north_m=0.0, east_m=0.0, alt_m=10.0,
                            tolerance_m=0.3, timeout_s=40.0)

    # ── Settle at hover ──
    # Hold position for 2s before switching to velocity mode.
    # This lets the drone damp any residual oscillation — same as AirSim's time.sleep(2)
    print("Settling at 10m...")
    t_settle = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - t_settle < 2.0:
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))
        await asyncio.sleep(0.05)

    # ── Switch to velocity setpoints for PID test ──
    # Send a zero velocity setpoint to transition from position to velocity mode
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(0.1)

    # ── Create PID expert and run test ──
    expert = PIDExpertHover(target_altitude=10.0)

    # Pre-load z integrals with current state before loop starts.
    # Without this, the integral is zero and the drone sinks for several
    # seconds while the integral slowly builds enough output to hold altitude.
    expert.reset(state=buf.get_state_dict())

    print(f"\nRunning PID hover test for 100 steps (5 seconds)...\n")

    positions = []
    for step in range(100):
        state  = buf.get_state_dict()
        action = expert.get_action(state)

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                float(action[0]),   # north
                float(action[1]),   # east
                float(action[2]),   # down (positive = descend)
                0.0
            )
        )

        alt = buf.altitude()
        positions.append(alt)

        if step % 20 == 0:
            dist = np.sqrt(buf.pos_n**2 + buf.pos_e**2)
            print(f"Step {step:3d}: Alt={alt:.2f}m, Dist from center={dist:.2f}m")

        await asyncio.sleep(expert.dt)

    # ── Stop offboard and land ──
    await drone.offboard.stop()
    await asyncio.sleep(0.5)
    await drone.action.land()

    # Statistics
    positions = np.array(positions)
    mean_alt  = np.mean(positions)
    std_alt   = np.std(positions)
    max_error = np.max(np.abs(positions - 10.0))

    print("\n" + "="*70)
    print("RESULTS")
    print("="*70)
    print(f"Mean altitude:  {mean_alt:.3f}m  (target: 10.0m)")
    print(f"Std deviation:  {std_alt:.3f}m")
    print(f"Max error:      {max_error:.3f}m")

    if std_alt < 0.3 and max_error < 0.5:
        print("\n  PID Expert EXCELLENT! Ready to collect demonstrations.")
    elif std_alt < 0.5:
        print("\n  PID Expert GOOD. Usable for demonstrations.")
    else:
        print("\n  PID Expert needs tuning. Adjust PID gains.")
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
    asyncio.run(test_pid_expert())
    os._exit(0)