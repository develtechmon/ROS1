"""
AIRSIM WIND VISUAL DEMO
========================
Arms, takes off to 10m, then applies wind from each direction
so you can watch the drone move in the AirSim window.

Sequence:
  1. Arm + takeoff to 10m
  2. Wind from North  (drone moves forward)
  3. Wind from South  (drone moves backward)
  4. Wind from East   (drone moves right)
  5. Wind from West   (drone moves left)
  6. Land

Run:
  Terminal 1: make px4_sitl_default none_iris
  Terminal 2: python wind_visual_demo.py
"""

import airsim
import asyncio
import threading
import time
import numpy as np
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw

# ── Shared state ──────────────────────────────────────────────
state = {
    'x': 0.0, 'y': 0.0, 'z': 0.0,
    'vx': 0.0, 'vy': 0.0,
    'ready': False
}
lock = threading.Lock()
cmd  = {'action': None}
done = threading.Event()


async def mavsdk_worker():
    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("[PX4] Connecting...")
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("[PX4] Connected")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[PX4] Position OK")
            break

    await drone.telemetry.set_rate_position_velocity_ned(20.0)

    async def update():
        async for p in drone.telemetry.position_velocity_ned():
            with lock:
                state['x']  =  p.position.north_m
                state['y']  =  p.position.east_m
                state['z']  = -p.position.down_m
                state['vx'] =  p.velocity.north_m_s
                state['vy'] =  p.velocity.east_m_s
                state['ready'] = True

    update_task = asyncio.ensure_future(update())

    while not state['ready']:
        await asyncio.sleep(0.1)

    # ── Arm + takeoff ──
    print("[PX4] Arming...")
    await drone.action.arm()
    await asyncio.sleep(1.0)

    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, 0, 0))
    await drone.offboard.start()

    print("[PX4] Climbing to 10m...")
    t0 = asyncio.get_event_loop().time()
    while True:
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -10.0, 0.0)
        )
        with lock:
            alt = state['z']
        if alt > 9.5:
            break
        if asyncio.get_event_loop().time() - t0 > 40.0:
            print("[PX4] Climb timeout")
            break
        await asyncio.sleep(0.05)

    # Settle 2s
    t1 = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - t1 < 2.0:
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -10.0, 0.0)
        )
        await asyncio.sleep(0.05)

    # Switch to velocity mode — drone holds zero velocity
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(0.5)

    with lock:
        alt = state['z']
    print(f"[PX4] Hovering at {alt:.2f}m — ready for wind demo")

    # ── Control loop ──
    while True:
        action = None
        with lock:
            action = cmd['action']
            if action:
                cmd['action'] = None

        if action == 'LAND':
            print("[PX4] Landing...")
            try:
                await drone.offboard.stop()
            except Exception:
                pass
            await drone.action.land()
            break

        # Keep sending zero velocity — wind will push against this
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.05)

    # Cancel telemetry task cleanly
    update_task.cancel()
    try:
        await update_task
    except asyncio.CancelledError:
        pass
    done.set()


def mavsdk_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(mavsdk_worker())
    loop.close()


def apply_wind(client, wx, wy, wz, label, duration=5.0):
    """Apply wind for a set duration and print live position."""
    print(f"\n{'='*55}")
    print(f"  WIND: {label}")
    print(f"  ({wx:+.0f}, {wy:+.0f}, {wz:+.0f}) m/s NED")
    print(f"  Duration: {duration}s — watch the AirSim window!")
    print(f"{'='*55}")

    with lock:
        x0 = state['x']
        y0 = state['y']

    client.simSetWind(airsim.Vector3r(wx, wy, wz))

    steps = int(duration / 0.5)
    print(f"  {'t':>5}  {'N pos':>8}  {'E pos':>8}  {'Alt':>7}  "
          f"{'vn':>7}  {'ve':>7}  {'drift':>7}")
    print("  " + "-"*57)

    for i in range(steps):
        time.sleep(0.5)
        with lock:
            x, y, z = state['x'], state['y'], state['z']
            vx, vy  = state['vx'], state['vy']
        drift = np.sqrt((x-x0)**2 + (y-y0)**2)
        print(f"  {(i+1)*0.5:>5.1f}s  {x:>+8.2f}  {y:>+8.2f}  "
              f"{z:>7.2f}  {vx:>+7.3f}  {vy:>+7.3f}  {drift:>7.2f}m")

    client.simSetWind(airsim.Vector3r(0, 0, 0))

    with lock:
        xf, yf = state['x'], state['y']
    total_drift = np.sqrt((xf-x0)**2 + (yf-y0)**2)
    print(f"\n  Total drift: {total_drift:.2f}m")
    print("  Wind cleared — drone stabilising 3s...")
    time.sleep(3.0)


def main():
    print("\n" + "="*55)
    print("AIRSIM WIND VISUAL DEMO")
    print("Arm → Takeoff → Wind from each direction")
    print("="*55)

    # ── Connect AirSim ──
    print("\nConnecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.simSetWind(airsim.Vector3r(0, 0, 0))
    print("  AirSim connected")

    # ── Start MAVSDK thread ──
    print("Starting PX4 control thread...")
    t = threading.Thread(target=mavsdk_thread, daemon=True)
    t.start()

    # Wait for telemetry
    print("Waiting for PX4 connection...")
    for _ in range(120):
        with lock:
            rdy = state['ready']
        if rdy:
            break
        time.sleep(0.5)

    # Wait for altitude
    print("Waiting for takeoff to 10m...")
    for _ in range(120):
        with lock:
            alt = state['z']
        if alt > 9.5:
            break
        time.sleep(0.5)

    # Extra settle time
    time.sleep(3.0)

    with lock:
        alt = state['z']
    print(f"\nDrone hovering at {alt:.2f}m")
    print("Starting wind demo in 2 seconds...")
    time.sleep(2.0)

    # ── Wind demo — four directions ──
    # NED frame: X=North, Y=East, Z=Down
    # Positive X wind → pushes drone North (forward in Unreal)
    # Positive Y wind → pushes drone East  (right in Unreal)

    apply_wind(client, +15.0,   0.0,  0.0,
               "FORWARD  — North 15 m/s  (X+)")

    apply_wind(client, -15.0,   0.0,  0.0,
               "BACKWARD — South 15 m/s  (X-)")

    apply_wind(client,   0.0, +15.0,  0.0,
               "RIGHT    — East  15 m/s  (Y+)")

    apply_wind(client,   0.0, -15.0,  0.0,
               "LEFT     — West  15 m/s  (Y-)")

    # ── Final summary ──
    print("\n" + "="*55)
    print("DEMO COMPLETE")
    print("Wind works from all four directions.")
    print("Ready to build Stage 2 training environment.")
    print("="*55)

    # Land
    print("\nLanding...")
    with lock:
        cmd['action'] = 'LAND'
    done.wait(timeout=20.0)
    print("Done")


if __name__ == "__main__":
    main()
