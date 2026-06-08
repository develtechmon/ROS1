import threading
"""
COLLECT EXPERT DEMONSTRATIONS - INDOOR VERSION (OPTICAL FLOW + RANGEFINDER, NO GPS)
=====================================================================================
Indoor adaptation of collect_demonstrations_mavsdk.py for MTF-01 use.
Use this when flying indoors. Use collect_demonstrations_mavsdk.py when flying outdoors.

HARDWARE:
  - Pixhawk (any variant)
  - MTF-01 optical flow + ToF rangefinder
  - NO GPS required

PX4 PARAMETERS — SET IN QGC BEFORE RUNNING (one-time setup):
  EKF2_OF_CTRL   = 1    Enable optical flow fusion
  EKF2_HGT_REF   = 2    Use rangefinder as height reference
  EKF2_RNG_AID   = 1    Range aid enabled
  EKF2_GPS_CTRL  = 0    No GPS indoors
  SYS_HAS_GPS    = 0    No GPS expected — prevents GPS-loss failsafe
  COM_ARM_WO_GPS = 1    Allow arming without GPS
  CBRK_FLIGHTTERM = 121212  Disable flight termination
  SENS_FLOW_ROT  = 0    No rotation on optical flow sensor

KEY DIFFERENCES FROM OUTDOOR VERSION:
  - Target altitude: 1.0m (not 10.0m) — safe for indoor ceiling
  - Health check: is_local_position_ok (not is_global_position_ok)
  - Horizontal start offsets: ±0.5m (not ±2m) — conservative for indoor space
  - Climb tolerance: 0.15m (not 0.4m) — tighter at low altitude
  - Climb timeout: 20s (not 40s) — only 1m to climb
  - Land wait: 3s (not 5s) — only 1m to fall
  - TARGET_D = buf.pos_d after settling (TARGET_D fix for ToF offset)

EPISODE TIMING (per episode):
  Arm + takeoff:     ~5s
  Hover collection:  max_steps × 0.05s = 25s (500 steps)
  Land + disarm:     ~5s
  Total:             ~35s per episode → 100 episodes ≈ 58 min

Usage (real Pixhawk via UDP proxy):
    python collect_demonstrations_indoor.py --episodes 100 --steps 500
    python collect_demonstrations_indoor.py --episodes 200 --steps 500

Usage (real Pixhawk via serial):
    python collect_demonstrations_indoor.py \
        --episodes 200 --steps 500 \
        --address serial:///dev/ttyACM0:921600

For quick test:
    python collect_demonstrations_indoor.py --episodes 10 --steps 200
"""

import asyncio
import numpy as np
import time
import pickle
import argparse
from pathlib import Path
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw

# Import from our translated expert file
from pid_expert_mavsdk import TelemetryBuffer
from pid_expert_mavsdk import subscribe_position, subscribe_attitude, subscribe_angular_velocity

# ── Indoor-specific constants ──
TARGET_ALT_M  = 1.0    # hover at 1m indoors (MTF-01 ToF is accurate here)
TARGET_D_NED  = -1.0   # NED: 1m altitude = -1.0 down
V_MAX         = 1.5    # reduced velocity limit for indoor safety
KP_N          = 0.5
KP_E          = 0.5
KP_D          = 2.0

# Safety abort thresholds — tighter for indoor
ABORT_ALT_MIN = 0.2    # abort if below 20cm
ABORT_ALT_MAX = 2.5    # abort if above 2.5m
ABORT_DIST    = 5.0    # abort if >5m from origin (small indoor space)


async def setup_drone(system_address="udp://:14550"):
    """Connect, wait for local position (optical flow), set telemetry rates, return drone + buffer."""
    drone = System()
    await drone.connect(system_address=system_address)
    print("  Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("  Connected!")
            break
    print("  Waiting for position estimate (optical flow + ToF)...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("  Local position OK (optical flow active)")
            break
    await drone.telemetry.set_rate_position_velocity_ned(20.0)
    await drone.telemetry.set_rate_attitude(20.0)
    buf = TelemetryBuffer()
    asyncio.ensure_future(subscribe_position(drone, buf))
    asyncio.ensure_future(subscribe_attitude(drone, buf))
    asyncio.ensure_future(subscribe_angular_velocity(drone, buf))
    while not buf.ready:
        await asyncio.sleep(0.1)
    print("  Telemetry buffer ready!")
    return drone, buf


async def wait_for_armed(drone, timeout_s=5.0):
    t0 = asyncio.get_event_loop().time()
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            return True
        if asyncio.get_event_loop().time() - t0 > timeout_s:
            return False
    return False


async def run_episode(drone, buf, max_steps, episode_num):
    episode_states  = []
    episode_actions = []
    episode_reward  = 0.0
    success         = True

    try:
        # ── Arm ──
        print(f"  [Ep {episode_num:3d}] Arming...", end=" ", flush=True)
        await drone.action.arm()
        armed = await wait_for_armed(drone, timeout_s=5.0)
        if not armed:
            print("FAILED (arm not confirmed)")
            await asyncio.sleep(2.0)
            return [], [], 0.0, False
        print("Armed", end=" | ", flush=True)

        # ── Small random start offset — conservative for indoor ──
        start_n = float(np.random.uniform(-0.5, 0.5))
        start_e = float(np.random.uniform(-0.5, 0.5))

        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
        except OffboardError as e:
            print(f"Offboard failed: {e}")
            await drone.action.disarm()
            await asyncio.sleep(2.0)
            return [], [], 0.0, False

        # ── Climb to 1m ──
        print("Climbing to 1m...", end=" ", flush=True)
        t_climb = asyncio.get_event_loop().time()
        while True:
            await drone.offboard.set_position_ned(
                PositionNedYaw(start_n, start_e, TARGET_D_NED, 0.0)
            )
            dd = buf.pos_d - TARGET_D_NED
            dn = buf.pos_n - start_n
            de = buf.pos_e - start_e
            if np.sqrt(dn**2 + de**2 + dd**2) <= 0.15:
                break
            if asyncio.get_event_loop().time() - t_climb > 20.0:
                print(f"(climb timeout at {buf.altitude():.2f}m)", end=" ", flush=True)
                break
            await asyncio.sleep(0.05)

        # ── Settle 1.5s ──
        t_settle = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - t_settle < 1.5:
            await drone.offboard.set_position_ned(
                PositionNedYaw(start_n, start_e, TARGET_D_NED, 0.0)
            )
            await asyncio.sleep(0.05)

        # ── Read actual settled position (TARGET_D fix) ──
        TARGET_N = buf.pos_n
        TARGET_E = buf.pos_e
        TARGET_D = buf.pos_d   # actual ToF altitude — not exactly -1.0

        print(f"At {buf.altitude():.2f}m | Collecting...", end=" ", flush=True)

        # ── Collect ──
        for step in range(max_steps):
            obs = buf.get_obs()

            await drone.offboard.set_position_ned(
                PositionNedYaw(TARGET_N, TARGET_E, TARGET_D, 0.0)
            )

            err_n  = TARGET_N - buf.pos_n
            err_e  = TARGET_E - buf.pos_e
            err_d  = TARGET_D - buf.pos_d

            vn_cmd = float(np.clip(KP_N * err_n, -V_MAX, V_MAX))
            ve_cmd = float(np.clip(KP_E * err_e, -V_MAX, V_MAX))
            vd_cmd = float(np.clip(KP_D * err_d, -V_MAX, V_MAX))

            action = np.array([vn_cmd, ve_cmd, vd_cmd], dtype=np.float32)

            alt              = buf.altitude()
            dist_from_center = np.sqrt(buf.pos_n**2 + buf.pos_e**2)
            reward           = 1.0 - abs(alt - TARGET_ALT_M) - dist_from_center * 0.5
            episode_reward  += reward

            episode_states.append(obs.copy())
            episode_actions.append(action.copy())

            # Tighter indoor safety limits
            if (dist_from_center > ABORT_DIST or
                    alt < ABORT_ALT_MIN or alt > ABORT_ALT_MAX):
                print(f"SAFETY ABORT (alt={alt:.2f}m dist={dist_from_center:.2f}m)")
                success = False
                break

            await asyncio.sleep(0.05)

        if success:
            print(f"Done ({len(episode_states)} samples, "
                  f"alt={buf.altitude():.2f}m, reward={episode_reward:.0f})")

        await drone.offboard.stop()
        await asyncio.sleep(0.3)

    except Exception as e:
        print(f"Exception: {e}")
        success = False

    finally:
        try:
            await drone.action.land()
            await asyncio.sleep(3.0)   # shorter — only 1m to fall
            await drone.action.disarm()
            await asyncio.sleep(1.5)
        except Exception:
            pass

    return episode_states, episode_actions, episode_reward, success


async def collect_demonstrations(num_episodes=100, max_steps=500,
                                  save_dir="./demonstrations_indoor",
                                  system_address="udp://:14550"):
    print("\n" + "="*70)
    print("COLLECTING INDOOR DEMONSTRATIONS — MTF-01 OPTICAL FLOW")
    print("="*70)
    print(f"Target altitude:  {TARGET_ALT_M}m (MTF-01 ToF range)")
    print(f"Episodes:         {num_episodes}")
    print(f"Steps/episode:    {max_steps}  ({max_steps*0.05:.0f}s each)")
    print(f"Expected samples: ~{num_episodes * max_steps:,}")
    print(f"Save directory:   {save_dir}")
    print()
    print("REQUIRED PX4 PARAMETERS:")
    print("  EKF2_HGT_REF  = 2  (range sensor)")
    print("  EKF2_RNG_AID  = 1  (range aid enabled)")
    print("  EKF2_GPS_CTRL = 0  (no GPS)")
    print("  EKF2_OF_CTRL  = 1  (optical flow)")
    print("="*70 + "\n")

    Path(save_dir).mkdir(parents=True, exist_ok=True)

    drone, buf = await setup_drone(system_address)

    all_states      = []
    all_actions     = []
    episode_rewards = []
    failed_episodes = 0

    start_time = time.time()

    for episode in range(num_episodes):
        ep_states, ep_actions, ep_reward, success = await run_episode(
            drone, buf, max_steps, episode + 1
        )

        if success and len(ep_states) > 0:
            all_states.extend(ep_states)
            all_actions.extend(ep_actions)
            episode_rewards.append(ep_reward)
        else:
            failed_episodes += 1

        elapsed     = time.time() - start_time
        eps_per_min = (episode + 1) / (elapsed / 60) if elapsed > 0 else 0
        eta_min     = (num_episodes - episode - 1) / eps_per_min if eps_per_min > 0 else 0
        avg_reward  = np.mean(episode_rewards[-10:]) if episode_rewards else 0
        print(f"Ep {episode+1:4d}/{num_episodes} | "
              f"Samples: {len(all_states):,} | "
              f"Avg reward: {avg_reward:6.1f} | "
              f"Speed: {eps_per_min:.1f} ep/min | "
              f"ETA: {eta_min:.0f} min | "
              f"Failed: {failed_episodes}")

        if (episode + 1) % 50 == 0 and len(all_states) > 0:
            cp = f"{save_dir}/checkpoint_{episode+1}.pkl"
            with open(cp, 'wb') as f:
                pickle.dump({'states': np.array(all_states),
                             'actions': np.array(all_actions),
                             'rewards': episode_rewards}, f)
            print(f"  Checkpoint: {cp} ({len(all_states):,} samples)")

    if not all_states:
        print("ERROR: No samples collected.")
        return None

    dataset = {
        'states':  np.array(all_states),
        'actions': np.array(all_actions),
        'rewards': episode_rewards,
        'metadata': {
            'num_episodes':    num_episodes,
            'failed_episodes': failed_episodes,
            'max_steps':       max_steps,
            'total_samples':   len(all_states),
            'target_alt_m':    TARGET_ALT_M,
            'sensor':          'MTF-01 optical flow + ToF',
            'environment':     'indoor',
            'platform':        'MAVSDK/PX4',
            'coordinate_frame': 'NED',
        }
    }

    path = f"{save_dir}/expert_demonstrations_indoor.pkl"
    with open(path, 'wb') as f:
        pickle.dump(dataset, f)

    print(f"\nSaved: {path}")
    print(f"Total samples: {len(all_states):,}")
    print(f"Time: {(time.time()-start_time)/60:.1f} min")
    return dataset


if __name__ == "__main__":
    import os
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes', type=int,   default=100)
    parser.add_argument('--steps',    type=int,   default=500)
    parser.add_argument('--save-dir', type=str,   default='./demonstrations_indoor')
    parser.add_argument('--address',  type=str,   default='udp://:14550')
    args = parser.parse_args()

    asyncio.run(collect_demonstrations(
        num_episodes=args.episodes,
        max_steps=args.steps,
        save_dir=args.save_dir,
        system_address=args.address,
    ))
    os._exit(0)
