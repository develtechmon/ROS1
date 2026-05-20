#!/usr/bin/env python3
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import AttitudeRate, PositionNedYaw, OffboardError

# Match MAVROS Gazebo working values
THRUST_BUMP      = 1.0    # same
THRUST_SPIN      = 0.55   # was 0.2 — KEY FIX, must be above hover (0.50)
THRUST_COUNTER   = 0.9    # same
COUNTER_DURATION = 0.20   # was 0.15
RECOVER_DURATION = 3.0    # was 4.0
FLIP_ALTITUDE    = 20.0   # same as MAVROS

#!/usr/bin/env python3
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import AttitudeRate, PositionNedYaw, OffboardError

# ── Values matched exactly from working MAVROS Gazebo code ───────────────────
ROLL_RATE         = math.degrees(30.0)   # 30 rad/s = 1718 deg/s
COUNTER_RATE      = math.degrees(50.0)   # 50 rad/s = 2865 deg/s
THRUST_BUMP       = 1.0
THRUST_SPIN       = 0.55    # KEY: must be > MPC_THR_HOVER (0.50) for Gazebo
THRUST_COUNTER    = 0.9
COUNTER_DURATION  = 0.20    # matched from MAVROS
RECOVER_DURATION  = 3.0     # matched from MAVROS
TAKEOFF_ALTITUDE  = 3.0
FLIP_ALTITUDE     = 20.0    # matched from MAVROS

INTENSITY = {
    'easy':   {'rate': math.degrees(10.0)},
    'medium': {'rate': math.degrees(20.0)},
    'hard':   {'rate': math.degrees(30.0)},
}

FLIP_CONFIGS = {
    'right':    {'roll':  ROLL_RATE, 'pitch':  0.0,        'counter_roll': -COUNTER_RATE, 'counter_pitch':  0.0},
    'left':     {'roll': -ROLL_RATE, 'pitch':  0.0,        'counter_roll':  COUNTER_RATE, 'counter_pitch':  0.0},
    'forward':  {'roll':  0.0,       'pitch':  ROLL_RATE,  'counter_roll':  0.0,          'counter_pitch': -COUNTER_RATE},
    'backward': {'roll':  0.0,       'pitch': -ROLL_RATE,  'counter_roll':  0.0,          'counter_pitch':  COUNTER_RATE},
}

keepalive_active = False
keepalive_paused = False
current_target_z = -TAKEOFF_ALTITUDE

async def keepalive(drone):
    global keepalive_active, keepalive_paused, current_target_z
    while keepalive_active:
        if not keepalive_paused:
            try:
                await drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, current_target_z, 0.0)
                )
            except Exception:
                pass
        await asyncio.sleep(0.05)

async def get_altitude(drone):
    async for pos in drone.telemetry.position():
        return pos.relative_altitude_m

async def get_euler(drone):
    async for euler in drone.telemetry.attitude_euler():
        return euler.roll_deg, euler.pitch_deg, euler.yaw_deg

async def get_quaternion(drone):
    async for quat in drone.telemetry.attitude_quaternion():
        return quat.w, quat.x, quat.y, quat.z

async def get_rates(drone):
    async for rates in drone.telemetry.attitude_angular_velocity_body():
        return rates.roll_rad_s, rates.pitch_rad_s, rates.yaw_rad_s

async def get_flight_mode(drone):
    async for mode in drone.telemetry.flight_mode():
        return mode

async def wait_altitude(drone, target_m, tolerance=0.8, timeout=60):
    print(f"  Waiting for altitude {target_m:.1f}m...")
    start = asyncio.get_event_loop().time()
    while True:
        alt = await get_altitude(drone)
        print(f"  alt={alt:.2f}m")
        if alt >= target_m - tolerance:
            print(f"  ✓ Reached {alt:.2f}m")
            return True
        if asyncio.get_event_loop().time() - start > timeout:
            print(f"  ⚠ Timeout at {alt:.2f}m")
            return False
        await asyncio.sleep(0.5)

async def wait_stable(drone, target_z, timeout=30):
    """Matched from MAVROS: wait for roll < 5° AND altitude near target."""
    print("  Waiting for stability...")
    start = asyncio.get_event_loop().time()
    while True:
        roll, pitch, _ = await get_euler(drone)
        vr, vp, _      = await get_rates(drone)
        _, _, z        = (0, 0, await get_altitude(drone))
        vr_deg         = math.degrees(vr)
        vp_deg         = math.degrees(vp)
        print(f"  roll={roll:.1f}°  pitch={pitch:.1f}°  z={z:.2f}m  "
              f"vr={vr_deg:.1f}°/s  vp={vp_deg:.1f}°/s")
        if (abs(roll)   < 5.0 and abs(pitch) < 5.0 and
            abs(vr_deg) < 2.0 and abs(vp_deg) < 2.0 and
            z > target_z - 2.0):
            print("  ✓ Stable!")
            return True
        if asyncio.get_event_loop().time() - start > timeout:
            print("  ⚠ Timeout")
            return False
        await asyncio.sleep(0.3)

async def ensure_offboard(drone):
    global keepalive_paused, current_target_z
    mode = await get_flight_mode(drone)
    if str(mode) != 'OFFBOARD':
        print(f"  ⚠ Mode={mode} — re-entering OFFBOARD...")
        keepalive_paused = False
        current_target_z = -FLIP_ALTITUDE
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, current_target_z, 0.0)
        )
        try:
            await drone.offboard.start()
        except OffboardError:
            pass
        await asyncio.sleep(1.0)

async def flip(drone, direction, intensity='hard'):
    global keepalive_paused, current_target_z

    cfg      = FLIP_CONFIGS[direction]
    lvl      = INTENSITY[intensity]
    is_pitch = cfg['roll'] == 0.0

    # Scale rate by intensity
    scale      = lvl['rate'] / ROLL_RATE
    roll_rate  = cfg['roll']          * scale
    pitch_rate = cfg['pitch']         * scale
    c_roll     = cfg['counter_roll']  * scale
    c_pitch    = cfg['counter_pitch'] * scale

    print(f"\n[FLIP {direction.upper()} | {intensity}]")
    await ensure_offboard(drone)

    # Pause keepalive — stop position hold fighting the flip
    keepalive_paused = True
    await asyncio.sleep(0.1)

    # Phase 1: Thrust bump — matched from MAVROS
    print("  Phase 1: Thrust bump (0.2s)...")
    await drone.offboard.set_attitude_rate(
        AttitudeRate(roll_deg_s=0.0, pitch_deg_s=0.0,
                     yaw_deg_s=0.0,  thrust_value=THRUST_BUMP)
    )
    await asyncio.sleep(0.2)

    # Phase 2: Spin — thrust 0.55 > hover (0.50), matched from MAVROS
    print(f"  Phase 2: Spinning {direction.upper()}...")
    await drone.offboard.set_attitude_rate(
        AttitudeRate(roll_deg_s=roll_rate, pitch_deg_s=pitch_rate,
                     yaw_deg_s=0.0,        thrust_value=THRUST_SPIN)
    )

    # Detection — break at 90° like MAVROS (not full 360°)
    # Works because thrust 0.55 keeps drone up during remaining rotation
    start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start < 3.0:
        if is_pitch:
            w, x, y, z = await get_quaternion(drone)
            print(f"  qw={w:.3f}  qy={y:.3f}")
            if abs(y) > 0.707:   # past 90° on pitch axis
                print("  ✓ Past 90°!")
                break
        else:
            roll, _, _ = await get_euler(drone)
            print(f"  roll={roll:.1f}°")
            if abs(math.radians(roll)) > math.pi / 2:
                print("  ✓ Past 90°!")
                break
        await asyncio.sleep(0.01)

    # Phase 3: Counter-rate — matched from MAVROS (0.20s, thrust 0.9)
    print("  Phase 3: Counter-rate (0.20s)...")
    await drone.offboard.set_attitude_rate(
        AttitudeRate(roll_deg_s=c_roll,  pitch_deg_s=c_pitch,
                     yaw_deg_s=0.0,      thrust_value=THRUST_COUNTER)
    )
    await asyncio.sleep(COUNTER_DURATION)

    # Phase 4: Position recovery — matched from MAVROS (3.0s)
    print("  Phase 4: Recovering...")
    current_target_z = -FLIP_ALTITUDE
    keepalive_paused  = False
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -FLIP_ALTITUDE, 0.0)
    )
    await asyncio.sleep(RECOVER_DURATION)

    roll, pitch, _ = await get_euler(drone)
    alt            = await get_altitude(drone)
    vr, vp, _      = await get_rates(drone)
    print(f"[FLIP {direction.upper()}] Done! "
          f"roll={roll:.1f}° pitch={pitch:.1f}° alt={alt:.2f}m "
          f"vr={math.degrees(vr):.1f}°/s vp={math.degrees(vp):.1f}°/s")

async def main():
    global keepalive_active, keepalive_paused, current_target_z

    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected!")
            break

    print("Waiting for armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("✓ Ready!")
            break

    # Verify clean start
    roll, pitch, _ = await get_euler(drone)
    vr, vp, _      = await get_rates(drone)
    print(f"Start: roll={roll:.1f}° pitch={pitch:.1f}° "
          f"vr={math.degrees(vr):.1f}°/s vp={math.degrees(vp):.1f}°/s")
    if abs(roll) > 5.0 or abs(pitch) > 5.0:
        print("⚠ Not level — restart PX4 SITL")
        return

    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
    await drone.action.arm()
    print("✓ Armed!")

    await drone.action.takeoff()
    await wait_altitude(drone, TAKEOFF_ALTITUDE)
    await asyncio.sleep(2.0)

    current_target_z = -TAKEOFF_ALTITUDE
    keepalive_paused  = False
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, current_target_z, 0.0)
    )
    try:
        await drone.offboard.start()
        print("✓ Offboard started!")
    except OffboardError as e:
        print(f"❌ {e}")
        await drone.action.land()
        return

    keepalive_active = True
    keepalive_task   = asyncio.create_task(keepalive(drone))

    # Climb — matched from MAVROS
    print(f"\nClimbing to {FLIP_ALTITUDE}m...")
    current_target_z = -FLIP_ALTITUDE
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, current_target_z, 0.0)
    )
    await wait_altitude(drone, FLIP_ALTITUDE)
    await asyncio.sleep(3.0)
    print("✓ At flip altitude. Ready.")

    FLIP_SEQUENCE  = ['right', 'left', 'forward', 'backward']
    FLIP_INTENSITY = 'hard'

    for i, direction in enumerate(FLIP_SEQUENCE):
        print(f"\n{'='*50}")
        print(f"Flip {i+1}/{len(FLIP_SEQUENCE)}: {direction.upper()} [{FLIP_INTENSITY}]")
        print(f"{'='*50}")

        await flip(drone, direction, intensity=FLIP_INTENSITY)

        # Wait stable after every flip including last
        await wait_stable(drone, target_z=FLIP_ALTITUDE)

        if i < len(FLIP_SEQUENCE) - 1:
            alt = await get_altitude(drone)
            if alt < FLIP_ALTITUDE - 3.0:
                print(f"  Re-climbing from {alt:.1f}m...")
                current_target_z = -FLIP_ALTITUDE
                keepalive_paused  = False
                await drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, current_target_z, 0.0)
                )
                await wait_altitude(drone, FLIP_ALTITUDE)
            await asyncio.sleep(3.0)

    print("\n✓ All flips done! Landing...")
    keepalive_active = False
    keepalive_task.cancel()
    await asyncio.sleep(0.5)
    await drone.offboard.stop()
    await asyncio.sleep(1.0)
    await drone.action.land()
    print("✓ Done!")

if __name__ == "__main__":
    asyncio.run(main())
