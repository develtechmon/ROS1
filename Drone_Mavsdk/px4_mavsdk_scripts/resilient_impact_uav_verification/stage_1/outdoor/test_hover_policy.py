import threading
"""
TEST HOVER POLICY - MAVSDK/PX4 VERSION
========================================
Tests the trained neural network policy in PX4 SITL (or HITL).
Direct translation of test_hover_policy_v2.py.

This uses the NEURAL NETWORK, not the PID expert.

KEY DIFFERENCE FROM AIRSIM VERSION:
  - No client.reset() — full land/disarm/arm/takeoff between episodes
  - State from async TelemetryBuffer, not synchronous getMultirotorState()
  - Actions via offboard.set_velocity_ned(), not moveByVelocityAsync()

Usage (SITL):
    Terminal 1: make px4_sitl gazebo
    Terminal 2: python test_hover_policy_mavsdk.py --model ./models/hover_policy_best.pth

                or run 
                python test_hover_policy.py --model ./models/hover_policy_best.pth --episodes 10


Usage (HITL):
    Connect Pixhawk via USB, set address to serial:
    python test_hover_policy_mavsdk.py --address serial:///dev/ttyUSB0:921600

Success criteria:
    80%+ episodes complete without safety abort → PASS
    Average distance from hover point < 0.5m  → EXCELLENT
"""

import asyncio
import torch
import torch.nn as nn
import numpy as np
import pickle
import argparse
import time
from pathlib import Path
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw

from pid_expert_mavsdk import TelemetryBuffer
from pid_expert_mavsdk import subscribe_position, subscribe_attitude, subscribe_angular_velocity


async def wait_for_armed(drone, timeout_s=5.0):
    """Wait until PX4 confirms armed state via telemetry — not just API ack."""
    t0 = asyncio.get_event_loop().time()
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            return True
        if asyncio.get_event_loop().time() - t0 > timeout_s:
            return False
    return False


# ─────────────────────────────────────────────
# Policy model (identical architecture to training)
# ─────────────────────────────────────────────
class HoverPolicy(nn.Module):
    def __init__(self, state_dim=13, action_dim=3):
        super(HoverPolicy, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(state_dim, 256), nn.ReLU(),
            nn.Linear(256, 256),       nn.ReLU(),
            nn.Linear(256, 128),       nn.ReLU(),
            nn.Linear(128, action_dim)
        )

    def predict(self, state_np):
        with torch.no_grad():
            t = torch.FloatTensor(state_np).unsqueeze(0)
            return self.network(t).squeeze(0).numpy()


# ─────────────────────────────────────────────
# Episode runner
# ─────────────────────────────────────────────
async def run_test_episode(drone, buf, model, max_steps, target_alt, episode_num):
    """
    Run one test episode with the neural network policy.

    Returns dict with success, steps, avg_distance, max_distance, reason.
    """
    distances  = []
    altitudes  = []
    success    = True
    reason     = "completed"
    step       = 0

    try:
        # Arm + verify via telemetry
        print(f"  [Ep {episode_num:2d}] Arming...", end=" ", flush=True)
        await drone.action.arm()
        armed = await wait_for_armed(drone, timeout_s=5.0)
        if not armed:
            print("FAILED")
            await asyncio.sleep(3.0)
            return {'episode': episode_num, 'success': False, 'steps': 0,
                    'avg_distance': 0, 'max_distance': 0,
                    'mean_altitude': 0, 'alt_std': 0, 'reason': 'arm_failed'}
        print("Armed", end=" | ", flush=True)

        # Enter offboard and climb to target altitude
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
        except OffboardError as e:
            print(f"Offboard failed: {e}")
            await drone.action.disarm()
            await asyncio.sleep(3.0)
            return {'episode': episode_num, 'success': False, 'steps': 0,
                    'avg_distance': 0, 'max_distance': 0,
                    'mean_altitude': 0, 'alt_std': 0, 'reason': 'offboard_failed'}

        print("Climbing...", end=" ", flush=True)
        t_start = asyncio.get_event_loop().time()
        while True:
            await drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -target_alt, 0.0)
            )
            if abs(buf.pos_d - (-target_alt)) <= 0.3:
                break
            if asyncio.get_event_loop().time() - t_start > 40.0:
                break
            await asyncio.sleep(0.05)

        # Settle
        t_settle = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - t_settle < 1.5:
            await drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -target_alt, 0.0)
            )
            await asyncio.sleep(0.05)

        # Switch to velocity mode
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)

        print(f"At {buf.altitude():.1f}m | Policy running...", flush=True)

        # Run neural network policy
        for step in range(max_steps):
            obs    = buf.get_obs()           # 13-dim NED state
            action = model.predict(obs)      # [vn, ve, vd]

            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(
                    float(action[0]),
                    float(action[1]),
                    float(action[2]),
                    0.0
                )
            )

            alt            = buf.altitude()
            dist_from_home = np.sqrt(buf.pos_n**2 + buf.pos_e**2)
            distances.append(dist_from_home)
            altitudes.append(alt)

            # Per-step print every 100 steps so you can watch behaviour
            if step % 100 == 0:
                print(f"    Step {step:3d}: Alt={alt:.2f}m | "
                      f"Dist={dist_from_home:.2f}m | "
                      f"Cmd=[{action[0]:.2f},{action[1]:.2f},{action[2]:.2f}]")

            # Safety termination
            if dist_from_home > 20.0:
                reason  = "out_of_bounds"
                success = False
                break
            if alt < 2.0 or alt > 30.0:
                reason  = "altitude_violation"
                success = False
                break

            await asyncio.sleep(0.05)

        await drone.offboard.stop()
        await asyncio.sleep(0.5)

    except Exception as e:
        print(f"    [Episode {episode_num}] Exception: {e}")
        success = False
        reason  = f"exception: {e}"

    finally:
        try:
            await drone.action.land()
            await asyncio.sleep(5.0)
            await drone.action.disarm()
            await asyncio.sleep(2.0)
        except Exception:
            pass

    avg_dist  = float(np.mean(distances)) if distances else 0.0
    max_dist  = float(np.max(distances))  if distances else 0.0
    mean_alt  = float(np.mean(altitudes)) if altitudes else 0.0
    alt_std   = float(np.std(altitudes))  if altitudes else 0.0

    return {
        'episode':       episode_num,
        'success':       success,
        'steps':         step + 1,
        'avg_distance':  avg_dist,
        'max_distance':  max_dist,
        'mean_altitude': mean_alt,
        'alt_std':       alt_std,
        'reason':        reason
    }


# ─────────────────────────────────────────────
# Main test runner
# ─────────────────────────────────────────────
async def test_policy(model_path, num_episodes=10, max_steps=500,
                      target_alt=10.0, system_address="udp://:14550"):

    print("\n" + "="*70)
    print("TESTING LEARNED HOVER POLICY - MAVSDK/PX4")
    print("="*70)
    print("Neural network inference — NOT the PID expert\n")

    # ── Load model ──
    print(f"[1/3] Loading model: {model_path}")
    model = HoverPolicy(state_dim=13, action_dim=3)
    model.load_state_dict(torch.load(model_path, map_location='cpu'))
    model.eval()
    print("   Model loaded")

    # Check model_info if available
    info_path = Path(model_path).parent / "model_info.pkl"
    if info_path.exists():
        with open(info_path, 'rb') as f:
            info = pickle.load(f)
        print(f"   Best val loss:    {info.get('best_val_loss', 'N/A'):.4f}")
        print(f"   Platform:         {info.get('platform', 'N/A')}")
        print(f"   Coord frame:      {info.get('coordinate_frame', 'N/A')}")
    print()

    # ── Connect ──
    print("[2/3] Connecting to drone...")
    drone = System()
    await drone.connect(system_address=system_address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("   Connected!")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("   Position estimate OK")
            break

    await drone.telemetry.set_rate_position_velocity_ned(20.0)
    await drone.telemetry.set_rate_attitude(20.0)
    

    buf = TelemetryBuffer()
    asyncio.ensure_future(subscribe_position(drone, buf))
    asyncio.ensure_future(subscribe_attitude(drone, buf))
    asyncio.ensure_future(subscribe_angular_velocity(drone, buf))

    while not buf.ready:
        await asyncio.sleep(0.1)
    print("   Telemetry ready\n")

    # ── Run episodes ──
    print(f"[3/3] Running {num_episodes} test episodes...")
    print(f"   Max steps/episode: {max_steps}")
    print(f"   Target altitude:   {target_alt}m\n")
    print("="*70)

    results = []
    for ep in range(1, num_episodes + 1):
        result = await run_test_episode(drone, buf, model, max_steps, target_alt, ep)
        results.append(result)

        status = "PASS" if result['success'] else "FAIL"
        print(f"  Episode {ep:2d}/{num_episodes} | "
              f"{'OK' if result['success'] else 'XX'} {status} | "
              f"Alt={result['mean_altitude']:.2f}±{result['alt_std']:.2f}m | "
              f"Dist={result['avg_distance']:.2f}m | "
              f"Steps={result['steps']} | "
              f"{result['reason']}")

    print("="*70)

    # ── Statistics ──
    print("\n" + "="*70)
    print("TEST RESULTS")
    print("="*70)

    successes    = sum(1 for r in results if r['success'])
    success_rate = successes / num_episodes * 100
    avg_steps    = np.mean([r['steps'] for r in results])

    print(f"Success Rate:     {success_rate:.0f}% ({successes}/{num_episodes})")
    print(f"Avg Episode Len:  {avg_steps:.1f} steps")

    if successes > 0:
        good = [r for r in results if r['success']]
        mean_alt_all = np.mean([r['mean_altitude'] for r in good])
        std_alt_all  = np.mean([r['alt_std']       for r in good])
        print(f"Mean Altitude:    {mean_alt_all:.3f}m  (target: {target_alt}m)")
        print(f"Altitude Std:     {std_alt_all:.3f}m")
        print(f"Altitude Error:   {abs(mean_alt_all - target_alt):.3f}m")
        print(f"Avg Dist (horiz): {np.mean([r['avg_distance'] for r in good]):.2f}m")
        print(f"Max Dist (horiz): {np.max([r['max_distance'] for r in good]):.2f}m")

    if successes < num_episodes:
        reasons = {}
        for r in results:
            if not r['success']:
                reasons[r['reason']] = reasons.get(r['reason'], 0) + 1
        print("\nFailure Reasons:")
        for reason, count in reasons.items():
            print(f"  {reason}: {count}")

    print()
    print("="*70)
    if success_rate >= 80:
        print("EXCELLENT! Policy successfully learned to hover on PX4!")
        print("  Ready for Stage 2 (transfer learning + disturbance)")
        print("  Model validated on MAVSDK/PX4 — HITL-ready")
    elif success_rate >= 60:
        print("GOOD. Policy works but not fully stable.")
        print("  Collect more demonstrations or increase epochs.")
    else:
        print("POOR. Policy failed.")
        print("  Check: (1) PID tuning, (2) demo quality, (3) coordinate frame signs")
    print("="*70 + "\n")

    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',    type=str,   default='./models/hover_policy_best.pth')
    parser.add_argument('--episodes', type=int,   default=10)
    parser.add_argument('--steps',    type=int,   default=500)
    parser.add_argument('--altitude', type=float, default=10.0)
    parser.add_argument('--address',  type=str,   default='udp://:14550',
                        help='SITL: udp://:14550 | HITL: serial:///dev/ttyUSB0:921600')
    args = parser.parse_args()

    # Simple, clean execution.
    # We use asyncio.run() and let os._exit(0) terminate the process cleanly.
    # This avoids the gRPC/asyncio "Event loop is closed" traceback that appears
    # in mavsdk 1.4.x on Python 3.8 when background telemetry streams are still
    # running at shutdown. The root cause: concurrent.futures catches the gRPC
    # callback exception and prints it directly to stderr before threading.excepthook
    # can intercept it. os._exit(0) terminates all threads immediately — the gRPC
    # callbacks never fire into a closed loop.
    import os
    asyncio.run(test_policy(
        model_path=args.model,
        num_episodes=args.episodes,
        max_steps=args.steps,
        target_alt=args.altitude,
        system_address=args.address
    ))
    os._exit(0)
