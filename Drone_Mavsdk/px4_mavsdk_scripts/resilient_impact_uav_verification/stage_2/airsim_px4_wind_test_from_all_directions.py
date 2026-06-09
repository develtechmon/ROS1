"""
AIRSIM WIND TEST v4 — correct Stage 2 architecture
====================================================
Key insight: for Stage 2 training, offboard mode stays ACTIVE
throughout. Wind is applied while the policy sends velocity commands.

The test shows:
  1. Drone in offboard VELOCITY mode sending zero commands
  2. Wind applied — drone drifts because zero velocity != position hold
  3. Drift measured — this is what Stage 2 PPO policy must learn to correct

This is the EXACT scenario Stage 2 trains on.
PX4 stays in OFFBOARD the whole time — no mode switching needed.

Run:
  Terminal 1: make px4_sitl_default none_iris
  Terminal 2: python test_wind_simple.py
"""

import airsim
import asyncio
import threading
import time
import numpy as np
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw, OffboardError

# ── Shared state ──────────────────────────────────────────────
state = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'vx': 0.0, 'vy': 0.0, 'ready': False}
lock  = threading.Lock()
cmd   = {'action': None}
done  = threading.Event()


async def mavsdk_worker():
    drone = System()
    await drone.connect(system_address="udp://:14550")

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

    # ── Arm and climb ──
    print("[PX4] Arming and climbing to 10m...")
    await drone.action.arm()
    await asyncio.sleep(1.0)

    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, 0, 0))
    await drone.offboard.start()

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
            break
        await asyncio.sleep(0.05)

    # Settle 2s at position
    t1 = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - t1 < 2.0:
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -10.0, 0.0)
        )
        await asyncio.sleep(0.05)

    # ── KEY: switch to VELOCITY mode ──
    # In velocity mode, sending (0,0,0) means "maintain zero velocity"
    # This is LESS aggressive than position hold
    # Wind will cause position drift which the policy must correct
    print("[PX4] Switching to velocity mode (zero command)...")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(0.5)
    print("[PX4] In velocity mode — ready for wind test")

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

        # Keep sending zero velocity — this is the baseline policy
        # In Stage 2, the PPO policy will send non-zero corrections here
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.05)

    # Cancel background telemetry task cleanly before loop exits
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


def main():
    print("\n" + "="*60)
    print("AIRSIM WIND TEST v4 — Stage 2 architecture")
    print("="*60)

    # ── Connect AirSim ──
    print("\n[1] Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.simSetWind(airsim.Vector3r(0, 0, 0))
    print("  Connected OK")

    # ── Start MAVSDK ──
    print("\n[2] Starting MAVSDK thread...")
    t = threading.Thread(target=mavsdk_thread, daemon=True)
    t.start()

    # Wait for ready
    for _ in range(120):
        with lock:
            rdy = state['ready']
        if rdy:
            break
        time.sleep(0.5)

    # Wait for altitude
    print("  Waiting for 10m...")
    for _ in range(120):
        with lock:
            alt = state['z']
        if alt > 9.5:
            break
        time.sleep(0.5)

    # Wait for velocity mode (extra 4s after altitude reached)
    time.sleep(4.0)

    with lock:
        x0 = state['x']
        y0 = state['y']
        a0 = state['z']

    # Record baseline — zero velocity command, no wind
    print(f"\n[3] Baseline at ({x0:+.2f}, {y0:+.2f}, {a0:.2f}m)")
    baseline_vx = []
    for _ in range(20):
        with lock:
            baseline_vx.append(state['vx'])
        time.sleep(0.1)
    base_vx = np.mean(baseline_vx)
    print(f"  Baseline vx={base_vx:+.4f} m/s (should be ~0)")

    # ── Apply wind — multiple directions and intensities ──
    # AirSim wind is in NED World frame:
    #   X = North (forward in Unreal)
    #   Y = East  (right in Unreal)
    #   Z = Down  (negative = upward wind)
    wind_scenarios = [
        (15.0,  0.0,  0.0, "North  15 m/s (from behind)"),
        (0.0,  15.0,  0.0, "East   15 m/s (from left)"),
        (-15.0, 0.0,  0.0, "South  15 m/s (headwind)"),
        (0.0, -15.0,  0.0, "West   15 m/s (from right)"),
        (10.0,  10.0, 0.0, "NE     14 m/s (diagonal)"),
        (20.0,  0.0,  0.0, "North  20 m/s (strong)"),
        (5.0,   0.0,  0.0, "North   5 m/s (light)"),
    ]

    all_results = []

    for wx, wy, wz, label in wind_scenarios:
        # Reset to origin position hold before each wind scenario
        print(f"\n  --- {label} ---")
        # Briefly command position hold to snap back to origin
        try:
            from mavsdk.offboard import PositionNedYaw as PNY
        except ImportError:
            pass

        # Record start position
        with lock:
            xs = state['x']
            ys = state['y']

        # Apply wind
        client.simSetWind(airsim.Vector3r(wx, wy, wz))
        time.sleep(0.2)

        # Measure drift over 5 seconds
        seg = []
        for i in range(50):
            with lock:
                x, y, z = state['x'], state['y'], state['z']
                vx, vy  = state['vx'], state['vy']
            seg.append((x, y, z, vx, vy))
            if i % 10 == 0:
                dx = x - xs
                dy = y - ys
                print(f"    t={i*0.1:.1f}s  "
                      f"N={x:+.2f}  E={y:+.2f}  Alt={z:.2f}  "
                      f"vn={vx:+.3f}  ve={vy:+.3f}  "
                      f"drift={np.sqrt(dx**2+dy**2):.2f}m")
            time.sleep(0.1)

        # Clear wind between scenarios
        client.simSetWind(airsim.Vector3r(0, 0, 0))
        time.sleep(1.0)   # let drone stabilise briefly

        max_d = max(np.sqrt((d[0]-xs)**2 + (d[1]-ys)**2) for d in seg)
        max_v = max(np.sqrt(d[3]**2 + d[4]**2) for d in seg)
        all_results.append((label, max_d, max_v))
        print(f"    Max drift={max_d:.2f}m  Max speed={max_v:.3f} m/s")

    print()
    max_drift    = max(r[1] for r in all_results)
    max_vx       = max(r[2] for r in all_results)
    final_drift  = all_results[0][1]   # first scenario for backward compat

    # ── Results ──
    print("\n[5] Summary")
    print("="*60)
    print(f"  {'Scenario':<30} {'Max drift':>10}  {'Max speed':>10}")
    print("  " + "-"*55)
    for label, md, mv in all_results:
        status = "PASS" if md > 0.1 else "WEAK"
        print(f"  {label:<30} {md:>9.2f}m  {mv:>9.3f} m/s  {status}")
    print("="*60)
    all_pass = all(r[1] > 0.1 for r in all_results)
    if all_pass:
        print("  OVERALL: PASS — Wind works from all directions")
        print("  Ready to build Stage 2 training environment.")
    else:
        print("  PARTIAL — Some directions not working")
    print("="*60)

    with lock:
        cmd['action'] = 'LAND'
    done.wait(timeout=15.0)
    print("Done")


if __name__ == "__main__":
    main()
