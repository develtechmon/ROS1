#!/usr/bin/env python3
"""
MAVSDK single flip script — PX4 SITL + AirSim
Single RIGHT flip only. Tune this until perfect, then add more.
"""
import asyncio
from mavsdk import System
from mavsdk.offboard import AttitudeRate, PositionNedYaw, OffboardError

# ── Config ────────────────────────────────────────────────────────────────────
ROLL_RATE        = 1718.0   # deg/s = 30 rad/s (matched from working MAVROS)
COUNTER_RATE     = 2865.0   # deg/s = 50 rad/s
THRUST_BUMP      = 1.0
THRUST_SPIN      = 0.2
THRUST_COUNTER   = 0.8
TAKEOFF_ALTITUDE = 3.0
FLIP_ALTITUDE    = 15.0

# ── Keepalive state ───────────────────────────────────────────────────────────
keepalive_active = False
keepalive_mode   = 'pos'
current_target_z = -TAKEOFF_ALTITUDE
current_rate_cmd = None

async def keepalive(drone):
    global keepalive_active, keepalive_mode, current_target_z, current_rate_cmd
    while keepalive_active:
        try:
            if keepalive_mode == 'pos':
                await drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, current_target_z, 0.0)
                )
            elif keepalive_mode == 'rate' and current_rate_cmd is not None:
                await drone.offboard.set_attitude_rate(current_rate_cmd)
        except Exception:
            pass
        await asyncio.sleep(0.05)

# ── Helpers ───────────────────────────────────────────────────────────────────
async def get_altitude(drone):
    async for pos in drone.telemetry.position():
        return pos.relative_altitude_m

async def get_euler(drone):
    async for euler in drone.telemetry.attitude_euler():
        return euler.roll_deg, euler.pitch_deg

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
            return
        if asyncio.get_event_loop().time() - start > timeout:
            print(f"  ⚠ Timeout at {alt:.2f}m")
            return
        await asyncio.sleep(0.5)

async def wait_stable(drone, timeout=30):
    """
    Wait until BOTH roll and pitch are settled AND angular rates are near zero.
    This is what was missing before — attitude angle can look stable
    but angular velocity still non-zero, causing crazy second flip.
    """
    print("  Waiting for stable attitude + zero rates...")
    start = asyncio.get_event_loop().time()
    while True:
        roll, pitch = await get_euler(drone)
        print(f"  roll={roll:.1f}°  pitch={pitch:.1f}°")
        if abs(roll) < 3.0 and abs(pitch) < 3.0:
            # Extra buffer — wait 2 more seconds after stable
            # to ensure angular rates fully damp
            print("  Attitude stable — waiting 2s for rate damping...")
            await asyncio.sleep(2.0)
            roll, pitch = await get_euler(drone)
            if abs(roll) < 3.0 and abs(pitch) < 3.0:
                print("  ✓ Fully stable!")
                return
        if asyncio.get_event_loop().time() - start > timeout:
            print("  ⚠ Stability timeout")
            return
        await asyncio.sleep(0.1)

async def flip_right(drone):
    global keepalive_mode, current_rate_cmd, current_target_z
    print("\n[FLIP RIGHT]")

    # Phase 1: Thrust bump
    print("  Phase 1: Thrust bump (0.2s)...")
    cmd = AttitudeRate(
        roll_deg_s=0.0, pitch_deg_s=0.0,
        yaw_deg_s=0.0,  thrust_value=THRUST_BUMP
    )
    current_rate_cmd = cmd
    keepalive_mode   = 'rate'
    await drone.offboard.set_attitude_rate(cmd)
    await asyncio.sleep(0.2)

    # Phase 2: Spin right
    print(f"  Phase 2: Spinning RIGHT at {ROLL_RATE:.0f} deg/s...")
    cmd = AttitudeRate(
        roll_deg_s=ROLL_RATE, pitch_deg_s=0.0,
        yaw_deg_s=0.0,        thrust_value=THRUST_SPIN
    )
    current_rate_cmd = cmd
    await drone.offboard.set_attitude_rate(cmd)

    # Wait until past 90°
    start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start < 5.0:
        roll, _ = await get_euler(drone)
        print(f"  roll={roll:.1f}°")
        if abs(roll) > 90.0:
            print("  ✓ Past 90°!")
            break
        await asyncio.sleep(0.01)

    # Phase 3: Counter-rate
    print("  Phase 3: Counter-rate (0.15s)...")
    cmd = AttitudeRate(
        roll_deg_s=-COUNTER_RATE, pitch_deg_s=0.0,
        yaw_deg_s=0.0,            thrust_value=THRUST_COUNTER
    )
    current_rate_cmd = cmd
    await drone.offboard.set_attitude_rate(cmd)
    await asyncio.sleep(0.15)

    # Phase 4: Recover position
    print(f"  Phase 4: Recovering to {FLIP_ALTITUDE}m...")
    current_target_z = -FLIP_ALTITUDE
    keepalive_mode   = 'pos'
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -FLIP_ALTITUDE, 0.0)
    )
    await asyncio.sleep(4.0)

    roll, pitch = await get_euler(drone)
    alt         = await get_altitude(drone)
    print(f"[FLIP RIGHT] Done! roll={roll:.1f}° pitch={pitch:.1f}° alt={alt:.2f}m")

# ── Main ──────────────────────────────────────────────────────────────────────
async def main():
    global keepalive_active, current_target_z, keepalive_mode

    drone = System()
    await drone.connect(system_address="udp://127.0.0.1:14550")

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

    # Arm + Takeoff
    print(f"\n[1] Setting takeoff altitude {TAKEOFF_ALTITUDE}m...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)

    print("\n[2] Arming...")
    await drone.action.arm()
    print("✓ Armed!")

    print(f"\n[3] Takeoff...")
    await drone.action.takeoff()
    await wait_altitude(drone, TAKEOFF_ALTITUDE)
    await asyncio.sleep(2.0)

    # Start offboard
    print("\n[4] Starting offboard...")
    current_target_z = -TAKEOFF_ALTITUDE
    keepalive_mode   = 'pos'
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, current_target_z, 0.0)
    )
    try:
        await drone.offboard.start()
        print("✓ Offboard started!")
    except OffboardError as e:
        print(f"❌ Offboard failed: {e}")
        await drone.action.land()
        return

    keepalive_active = True
    keepalive_task   = asyncio.create_task(keepalive(drone))
    print("✓ Keepalive running!")

    # Climb
    print(f"\n[5] Climbing to {FLIP_ALTITUDE}m...")
    current_target_z = -FLIP_ALTITUDE
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, current_target_z, 0.0)
    )
    await wait_altitude(drone, FLIP_ALTITUDE)
    await asyncio.sleep(3.0)

    # Single flip
    await flip_right(drone)

    # Wait fully stable after flip
    await wait_stable(drone)

    # Land
    print("\n[6] Landing...")
    keepalive_active = False
    keepalive_task.cancel()
    await asyncio.sleep(0.5)
    await drone.offboard.stop()
    await asyncio.sleep(1.0)
    await drone.action.land()
    print("✓ Done!")

if __name__ == "__main__":
    asyncio.run(main())
