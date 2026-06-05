#!/usr/bin/env python3
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import AttitudeRate, PositionNedYaw, OffboardError

ROLL_RATE        = 1718.0
COUNTER_RATE     = 2865.0
THRUST_BUMP      = 1.0
THRUST_SPIN      = 0.2
THRUST_COUNTER   = 0.8
TAKEOFF_ALTITUDE = 3.0
FLIP_ALTITUDE    = 25.0

# Intensity levels for disturbance injection
# Tune these for your PPO curriculum stages
INTENSITY = {
    'easy':   {'rate': 500.0,    'duration': 0.15},
    'medium': {'rate': 1000.0,   'duration': 0.20},
    'hard':   {'rate': ROLL_RATE, 'duration': 0.25},
}

FLIP_CONFIGS = {
    'right':    {'roll':  ROLL_RATE, 'pitch':  0.0,       'counter_roll': -COUNTER_RATE, 'counter_pitch':  0.0},
    'left':     {'roll': -ROLL_RATE, 'pitch':  0.0,       'counter_roll':  COUNTER_RATE, 'counter_pitch':  0.0},
    'forward':  {'roll':  0.0,       'pitch':  ROLL_RATE, 'counter_roll':  0.0,          'counter_pitch': -COUNTER_RATE},
    'backward': {'roll':  0.0,       'pitch': -ROLL_RATE, 'counter_roll':  0.0,          'counter_pitch':  COUNTER_RATE},
}

# ── Keepalive ─────────────────────────────────────────────────────────────────
keepalive_active  = False
keepalive_paused  = False   # pause during flip so it doesn't fight the maneuver
current_target_z  = -TAKEOFF_ALTITUDE

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

# ── Telemetry helpers ─────────────────────────────────────────────────────────
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

# ── Flight helpers ────────────────────────────────────────────────────────────
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

async def wait_stable(drone, timeout=40):
    print("  Waiting for stable...")
    start = asyncio.get_event_loop().time()
    while True:
        roll, pitch, _ = await get_euler(drone)
        vr, vp, _      = await get_rates(drone)
        vr_deg         = math.degrees(vr)
        vp_deg         = math.degrees(vp)
        print(f"  roll={roll:.1f}°  pitch={pitch:.1f}°  "
              f"vr={vr_deg:.1f}°/s  vp={vp_deg:.1f}°/s")
        if (abs(roll)   < 5.0 and abs(pitch) < 5.0 and
            abs(vr_deg) < 2.0 and abs(vp_deg) < 2.0):
            await asyncio.sleep(2.0)
            roll, pitch, _ = await get_euler(drone)
            vr, vp, _      = await get_rates(drone)
            if (abs(roll)             < 5.0 and
                abs(pitch)            < 5.0 and
                abs(math.degrees(vr)) < 2.0 and
                abs(math.degrees(vp)) < 2.0):
                print("  ✓ Stable!")
                return True
        if asyncio.get_event_loop().time() - start > timeout:
            print("  ⚠ Timeout")
            return False
        await asyncio.sleep(0.1)

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

# ── Core flip function ────────────────────────────────────────────────────────
async def flip(drone, direction, intensity='hard'):
    """
    Flip the drone in given direction at given intensity.

    direction: 'right', 'left', 'forward', 'backward'
    intensity: 'easy', 'medium', 'hard'

    For PPO disturbance injection:
    - easy   → small tilt (~30°), drone can recover easily
    - medium → moderate flip (~90°-180°), moderate recovery needed
    - hard   → full 360° flip, maximum recovery challenge
    """
    global keepalive_paused, current_target_z

    cfg      = FLIP_CONFIGS[direction]
    lvl      = INTENSITY[intensity]
    is_pitch = cfg['roll'] == 0.0

    # Scale rate by intensity
    roll_rate  = cfg['roll']  / ROLL_RATE * lvl['rate'] if cfg['roll']  != 0.0 else 0.0
    pitch_rate = cfg['pitch'] / ROLL_RATE * lvl['rate'] if cfg['pitch'] != 0.0 else 0.0
    c_roll     = cfg['counter_roll']  / COUNTER_RATE * lvl['rate'] if cfg['counter_roll']  != 0.0 else 0.0
    c_pitch    = cfg['counter_pitch'] / COUNTER_RATE * lvl['rate'] if cfg['counter_pitch'] != 0.0 else 0.0

    print(f"\n[FLIP {direction.upper()} | {intensity}] "
          f"roll_rate={roll_rate:.0f} pitch_rate={pitch_rate:.0f} deg/s")
    await ensure_offboard(drone)

    # ── PAUSE keepalive during flip ───────────────────────────────────────────
    # This is the critical fix — position hold was fighting the flip
    keepalive_paused = True
    await asyncio.sleep(0.1)  # let any in-flight keepalive message finish

    # Phase 1: Thrust bump
    print("  Phase 1: Thrust bump (0.2s)...")
    await drone.offboard.set_attitude_rate(
        AttitudeRate(roll_deg_s=0.0, pitch_deg_s=0.0,
                     yaw_deg_s=0.0,  thrust_value=THRUST_BUMP)
    )
    await asyncio.sleep(0.2)

    # Phase 2: Spin
    print(f"  Phase 2: Spinning {direction.upper()}...")
    await drone.offboard.set_attitude_rate(
        AttitudeRate(roll_deg_s=roll_rate, pitch_deg_s=pitch_rate,
                     yaw_deg_s=0.0,        thrust_value=THRUST_SPIN)
    )

    # Flip detection
    # right/left  → Euler roll (reliable, no gimbal lock)
    # forward/back → quaternion q.y (Euler pitch locks at ±90°)
    past_90 = False
    start   = asyncio.get_event_loop().time()

    while asyncio.get_event_loop().time() - start < 5.0:
        if is_pitch:
            w, x, y, z = await get_quaternion(drone)
            print(f"  qw={w:.3f}  qy={y:.3f}  past_90={past_90}")
            if not past_90:
                if abs(y) > 0.707:
                    past_90 = True
                    print("  ✓ Past 90°!")
            else:
                if w > 0.98 and abs(y) < 0.2:
                    print("  ✓ 360° complete!")
                    break
        else:
            roll, _, _ = await get_euler(drone)
            angle_rad  = math.radians(roll)
            print(f"  roll={roll:.1f}°  past_90={past_90}")
            if not past_90:
                if abs(angle_rad) > math.pi / 2:
                    past_90 = True
                    print("  ✓ Past 90°!")
            else:
                if abs(angle_rad) < 0.2:
                    print("  ✓ 360° complete!")
                    break
        await asyncio.sleep(0.01)
    else:
        print("  ⚠ Timeout — applying counter anyway")

    # Phase 3: Counter-rate
    print("  Phase 3: Counter-rate (0.15s)...")
    await drone.offboard.set_attitude_rate(
        AttitudeRate(roll_deg_s=c_roll, pitch_deg_s=c_pitch,
                     yaw_deg_s=0.0,    thrust_value=THRUST_COUNTER)
    )
    await asyncio.sleep(0.15)

    # ── RESUME keepalive for position recovery ────────────────────────────────
    print("  Phase 4: Position hold...")
    current_target_z = -FLIP_ALTITUDE
    keepalive_paused  = False
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -FLIP_ALTITUDE, 0.0)
    )
    await asyncio.sleep(4.0)

    roll, pitch, _ = await get_euler(drone)
    alt            = await get_altitude(drone)
    vr, vp, _      = await get_rates(drone)
    print(f"[FLIP {direction.upper()}] Done! "
          f"roll={roll:.1f}° pitch={pitch:.1f}° alt={alt:.2f}m "
          f"vr={math.degrees(vr):.1f}°/s vp={math.degrees(vp):.1f}°/s")

# ── Main ──────────────────────────────────────────────────────────────────────
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
    print(f"Start state: roll={roll:.1f}° pitch={pitch:.1f}° "
          f"vr={math.degrees(vr):.1f}°/s vp={math.degrees(vp):.1f}°/s")
    if abs(roll) > 5.0 or abs(pitch) > 5.0:
        print("⚠ Drone not level — restart PX4 SITL")
        return

    # Arm + takeoff
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
    await drone.action.arm()
    print("✓ Armed!")

    await drone.action.takeoff()
    await wait_altitude(drone, TAKEOFF_ALTITUDE)
    await asyncio.sleep(2.0)

    # Start offboard + keepalive
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
    print("✓ Keepalive running!")

    # Climb to flip altitude
    print(f"\nClimbing to {FLIP_ALTITUDE}m...")
    current_target_z = -FLIP_ALTITUDE
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, current_target_z, 0.0)
    )
    await wait_altitude(drone, FLIP_ALTITUDE)
    await asyncio.sleep(3.0)
    print("✓ At flip altitude. Ready.")

    # Flip sequence — change intensity here for PPO curriculum
    FLIP_SEQUENCE = ['right', 'left', 'forward', 'backward']
    FLIP_INTENSITY = 'hard'   # 'easy', 'medium', 'hard'

    for i, direction in enumerate(FLIP_SEQUENCE):
        print(f"\n{'='*50}")
        print(f"Flip {i+1}/{len(FLIP_SEQUENCE)}: {direction.upper()} [{FLIP_INTENSITY}]")
        print(f"{'='*50}")

        await flip(drone, direction, intensity=FLIP_INTENSITY)

        # Wait stable after every flip including the last
        await wait_stable(drone)

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
