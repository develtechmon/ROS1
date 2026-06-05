import threading
"""
COLLECT EXPERT DEMONSTRATIONS - MAVSDK/PX4 VERSION
====================================================
Direct translation of collect_demonstration_v2.py from AirSim to MAVSDK.

KEY DIFFERENCES FROM AIRSIM VERSION:
  - No client.reset() — episode resets via: land → disarm → reposition → arm → takeoff
  - Telemetry is async streaming, collected via TelemetryBuffer
  - Each episode takes longer (~25-30s) due to real arm/takeoff/land cycle
  - No random start position offset (PX4 doesn't support teleport)
    → We vary initial hover time instead to get different initial conditions

EPISODE TIMING (per episode):
  Arm + takeoff:     ~8s
  Hover collection:  max_steps × 0.05s = 10s (200 steps)
  Land + disarm:     ~8s
  Total:             ~26s per episode → ~200 episodes ≈ 87 min

For SITL, you can speed this up by reducing takeoff/land wait times.
For HITL with real hardware, use the full times.

Usage:
    Terminal 1: make px4_sitl gazebo
    Terminal 2: python collect_demonstrations_mavsdk.py --episodes 50 --steps 200

For quick test:
    python collect_demonstrations_mavsdk.py --episodes 10 --steps 200
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
from pid_expert_outdoor_mavsdk import PIDExpertHover, TelemetryBuffer
from pid_expert_outdoor_mavsdk import subscribe_position, subscribe_attitude, subscribe_angular_velocity


async def setup_drone(system_address="udp://:14550"):
    """Connect, wait for health, set telemetry rates, return drone + buffer."""
    drone = System()
    await drone.connect(system_address=system_address)

    print("  Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("  Connected!")
            break

    print("  Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("  Position estimate OK")
            break

    # Set telemetry rates to 20Hz
    # NOTE: set_rate_attitude() covers quaternion, euler, AND angular velocity in mavsdk 1.4.x
    await drone.telemetry.set_rate_position_velocity_ned(20.0)
    await drone.telemetry.set_rate_attitude(20.0)

    # Create and start telemetry buffer
    buf = TelemetryBuffer()
    asyncio.ensure_future(subscribe_position(drone, buf))
    asyncio.ensure_future(subscribe_attitude(drone, buf))
    asyncio.ensure_future(subscribe_angular_velocity(drone, buf))

    print("  Waiting for telemetry buffer...")
    while not buf.ready:
        await asyncio.sleep(0.1)
    print("  Telemetry buffer ready!")

    return drone, buf


async def wait_for_armed(drone, timeout_s=5.0):
    """
    Wait until PX4 confirms the drone is actually armed.
    action.arm() returns before arming completes — we must verify.
    Without this check, offboard starts on a disarmed drone and the
    climb loop runs forever (drone never moves).
    """
    t0 = asyncio.get_event_loop().time()
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            return True
        if asyncio.get_event_loop().time() - t0 > timeout_s:
            return False
    return False


async def run_episode(drone, buf, expert, max_steps, episode_num):
    """
    Run one data collection episode.

    EPISODE FLOW:
        1. Arm  (verified via telemetry, not just API call)
        2. Offboard position climb to random (±2m N/E, 10m alt)
        3. Settle 1.5s at hover
        4. Switch to velocity setpoints → collect (obs, action) pairs
        5. Stop offboard → land → disarm
    """
    episode_states  = []
    episode_actions = []
    episode_reward  = 0.0
    success         = True

    try:
        # ── Step 1: Arm + verify ──
        print(f"  [Ep {episode_num:3d}] Arming...", end=" ", flush=True)
        await drone.action.arm()

        # Confirm arm via telemetry — API call returns before FC confirms
        armed = await wait_for_armed(drone, timeout_s=5.0)
        if not armed:
            print(f"FAILED (arm not confirmed)")
            await asyncio.sleep(3.0)
            return [], [], 0.0, False
        print(f"Armed", end=" | ", flush=True)

        # ── Step 2: Offboard + climb ──
        start_n = float(np.random.uniform(-2, 2))
        start_e = float(np.random.uniform(-2, 2))

        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
        except OffboardError as e:
            print(f"Offboard failed: {e}")
            await drone.action.disarm()
            await asyncio.sleep(3.0)
            return [], [], 0.0, False

        print(f"Climbing...", end=" ", flush=True)
        t_climb = asyncio.get_event_loop().time()
        while True:
            await drone.offboard.set_position_ned(
                PositionNedYaw(start_n, start_e, -10.0, 0.0)
            )
            dn = buf.pos_n - start_n
            de = buf.pos_e - start_e
            dd = buf.pos_d - (-10.0)
            if np.sqrt(dn**2 + de**2 + dd**2) <= 0.4:
                break
            if asyncio.get_event_loop().time() - t_climb > 40.0:
                print(f"(climb timeout at {buf.altitude():.1f}m)", end=" ", flush=True)
                break
            await asyncio.sleep(0.05)

        # ── Settle ──
        t_settle = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - t_settle < 1.5:
            await drone.offboard.set_position_ned(
                PositionNedYaw(start_n, start_e, -10.0, 0.0)
            )
            await asyncio.sleep(0.05)

        # Switch to velocity for PID
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)
        expert.reset()

        print(f"At {buf.altitude():.1f}m | Collecting...", end=" ", flush=True)

        # ── Step 3: Collect ──
        for step in range(max_steps):
            state  = buf.get_state_dict()
            obs    = buf.get_obs()
            action = expert.get_action(state)

            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(
                    float(action[0]),
                    float(action[1]),
                    float(action[2]),
                    0.0
                )
            )

            alt              = buf.altitude()
            dist_from_center = np.sqrt(buf.pos_n**2 + buf.pos_e**2)
            reward           = 10.0 - abs(alt - 10.0) - dist_from_center * 0.5
            episode_reward  += reward

            episode_states.append(obs.copy())
            episode_actions.append(action.copy())

            if dist_from_center > 15.0 or alt < 2.0 or alt > 25.0:
                print(f"SAFETY ABORT (alt={alt:.1f}m dist={dist_from_center:.1f}m)")
                success = False
                break

            await asyncio.sleep(expert.dt)

        if success:
            print(f"Done ({len(episode_states)} samples, "
                  f"alt={buf.altitude():.1f}m, reward={episode_reward:.0f})")

        # ── Step 4: Stop offboard ──
        await drone.offboard.stop()
        await asyncio.sleep(0.5)

    except Exception as e:
        print(f"Exception: {e}")
        success = False

    finally:
        # ── Step 5: Land + disarm (always runs) ──
        try:
            await drone.action.land()
            await asyncio.sleep(5.0)
            await drone.action.disarm()
            await asyncio.sleep(2.0)
        except Exception:
            pass

    return episode_states, episode_actions, episode_reward, success


async def collect_demonstrations(num_episodes=200, max_steps=200,
                                  save_dir="./demonstrations",
                                  system_address="udp://:14550"):
    print("\n" + "="*70)
    print("COLLECTING EXPERT DEMONSTRATIONS - MAVSDK/PX4 VERSION")
    print("="*70)
    print(f"Target episodes:  {num_episodes}")
    print(f"Steps/episode:    {max_steps}")
    print(f"Expected samples: ~{num_episodes * max_steps:,}")
    print(f"Est. time (SITL): ~{num_episodes * 26 / 60:.0f} min")
    print(f"Save directory:   {save_dir}")
    print("="*70 + "\n")

    Path(save_dir).mkdir(parents=True, exist_ok=True)

    print("Setting up drone connection...")
    drone, buf = await setup_drone(system_address)

    expert = PIDExpertHover(target_altitude=10.0)

    all_states = []
    all_actions = []
    episode_rewards = []
    failed_episodes = 0

    print("\nStarting collection...\n")
    start_time = time.time()

    for episode in range(num_episodes):
        ep_states, ep_actions, ep_reward, success = await run_episode(
            drone, buf, expert, max_steps, episode + 1
        )

        if success and len(ep_states) > 0:
            all_states.extend(ep_states)
            all_actions.extend(ep_actions)
            episode_rewards.append(ep_reward)
        else:
            failed_episodes += 1

        # Progress log every episode
        if True:
            elapsed = time.time() - start_time
            completed = episode + 1 - failed_episodes
            eps_per_min = (episode + 1) / (elapsed / 60) if elapsed > 0 else 0
            eta_min = (num_episodes - episode - 1) / eps_per_min if eps_per_min > 0 else 0
            avg_reward = np.mean(episode_rewards[-10:]) if episode_rewards else 0

            print(f"Episode {episode+1:4d}/{num_episodes} | "
                  f"Collected: {completed} | "
                  f"Avg Reward: {avg_reward:7.1f} | "
                  f"Speed: {eps_per_min:.1f} eps/min | "
                  f"ETA: {eta_min:.0f} min | "
                  f"Failed: {failed_episodes}")

        # Checkpoint every 50 episodes
        if (episode + 1) % 50 == 0 and len(all_states) > 0:
            cp_path = f"{save_dir}/checkpoint_{episode+1}.pkl"
            cp_data = {
                'states': np.array(all_states),
                'actions': np.array(all_actions),
                'rewards': episode_rewards
            }
            with open(cp_path, 'wb') as f:
                pickle.dump(cp_data, f)
            print(f"  Checkpoint saved: {cp_path} ({len(all_states):,} samples)")

    # ── Final save ──
    print("\n" + "="*70)
    print("SAVING FINAL DATASET")
    print("="*70)

    if len(all_states) == 0:
        print("ERROR: No samples collected! Check SITL connection.")
        return None

    dataset = {
        'states': np.array(all_states),
        'actions': np.array(all_actions),
        'rewards': episode_rewards,
        'metadata': {
            'num_episodes':      num_episodes,
            'failed_episodes':   failed_episodes,
            'max_steps':         max_steps,
            'total_samples':     len(all_states),
            'state_dim':         all_states[0].shape[0],
            'action_dim':        all_actions[0].shape[0],
            'collection_time_s': time.time() - start_time,
            'platform':          'MAVSDK/PX4',
            'coordinate_frame':  'NED'
        }
    }

    final_path = f"{save_dir}/expert_demonstrations_mavsdk.pkl"
    with open(final_path, 'wb') as f:
        pickle.dump(dataset, f)

    print(f"\nDataset Statistics:")
    print(f"  Total samples:     {len(all_states):,}")
    print(f"  State dimension:   {all_states[0].shape[0]}  (should be 13)")
    print(f"  Action dimension:  {all_actions[0].shape[0]} (should be 3)")
    print(f"  Successful eps:    {len(episode_rewards)}/{num_episodes}")
    print(f"  Failed episodes:   {failed_episodes}")
    if episode_rewards:
        print(f"  Mean ep reward:    {np.mean(episode_rewards):.1f}")
        print(f"  Std ep reward:     {np.std(episode_rewards):.1f}")
    print(f"  Collection time:   {(time.time() - start_time)/60:.1f} min")
    print(f"\n  Saved to: {final_path}")
    print(f"  File size: {Path(final_path).stat().st_size / 1e6:.1f} MB")
    print("="*70 + "\n")

    return dataset


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes',  type=int,   default=50,
                        help='Number of episodes (default 50 for SITL test)')
    parser.add_argument('--steps',     type=int,   default=200,
                        help='Steps per episode')
    parser.add_argument('--save-dir',  type=str,   default='./demonstrations')
    parser.add_argument('--address',   type=str,   default='udp://:14550',
                        help='MAVSDK system address')
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
    asyncio.run(collect_demonstrations(
        num_episodes=args.episodes,
        max_steps=args.steps,
        save_dir=args.save_dir,
        system_address=args.address
    ))
    os._exit(0)