import threading
"""
STAGE 1 STATISTICS — INDOOR VERSION (OPTICAL FLOW + RANGEFINDER, NO GPS)
==========================================================================
Indoor adaptation of stage1_statistics.py for MTF-01 use.
Use this when evaluating indoors. Use stage1_statistics.py for outdoor/GPS.

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
  - Target altitude: 1.0m (not 10.0m)
  - Health check: is_local_position_ok (not is_global_position_ok)
  - Start offsets: ±0.5m (not ±2m) — conservative for indoor space
  - Safety limits: alt < 0.2m or > 2.5m = abort
  - Saved to separate results folder to preserve outdoor results

Usage (real Pixhawk via UDP proxy):
    python stage1_statistics_indoor.py \
        --model ./models/hover_policy_indoor_best.pth \
        --episodes 10 --steps 500 \
        --results ./results/indoor

Usage (real Pixhawk via serial):
    python stage1_statistics_indoor.py \
        --model ./models/hover_policy_indoor_best.pth \
        --episodes 10 --steps 500 \
        --address serial:///dev/ttyACM0:921600 \
        --results ./results/indoor
"""

import asyncio
import torch
import torch.nn as nn
import numpy as np
import pickle
import argparse
import time
import os
from pathlib import Path
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw

from pid_expert_mavsdk import (
    TelemetryBuffer,
    subscribe_position,
    subscribe_attitude,
    subscribe_angular_velocity,
)

TARGET_ALT_M = 1.0
TARGET_D_NED = -1.0


class HoverPolicy(nn.Module):
    def __init__(self, state_dim=13, action_dim=3):
        super().__init__()
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


async def wait_for_armed(drone, timeout_s=5.0):
    t0 = asyncio.get_event_loop().time()
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            return True
        if asyncio.get_event_loop().time() - t0 > timeout_s:
            return False
    return False


async def connect_drone(system_address):
    drone = System()
    await drone.connect(system_address=system_address)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("  Connected!")
            break
    print("  Waiting for local position (optical flow)...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("  Local position OK")
            break
    await drone.telemetry.set_rate_position_velocity_ned(20.0)
    await drone.telemetry.set_rate_attitude(20.0)
    buf = TelemetryBuffer()
    asyncio.ensure_future(subscribe_position(drone, buf))
    asyncio.ensure_future(subscribe_attitude(drone, buf))
    asyncio.ensure_future(subscribe_angular_velocity(drone, buf))
    while not buf.ready:
        await asyncio.sleep(0.1)
    print("  Telemetry ready\n")
    return drone, buf


async def arm_and_climb(drone, buf, start_n, start_e):
    await drone.action.arm()
    armed = await wait_for_armed(drone, timeout_s=5.0)
    if not armed:
        return False, None
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError:
        return False, None
    t0 = asyncio.get_event_loop().time()
    while True:
        await drone.offboard.set_position_ned(
            PositionNedYaw(start_n, start_e, TARGET_D_NED, 0.0)
        )
        dd = abs(buf.pos_d - TARGET_D_NED)
        if dd <= 0.10:   # tighter tolerance at 1m
            break
        if asyncio.get_event_loop().time() - t0 > 20.0:
            break
        await asyncio.sleep(0.05)
    # Settle
    t1 = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - t1 < 1.5:
        await drone.offboard.set_position_ned(
            PositionNedYaw(start_n, start_e, TARGET_D_NED, 0.0)
        )
        await asyncio.sleep(0.05)
    actual_pos_d = buf.pos_d
    return True, actual_pos_d


async def land_and_disarm(drone):
    try:
        await drone.offboard.stop()
        await asyncio.sleep(0.3)
    except Exception:
        pass
    try:
        await drone.action.land()
        await asyncio.sleep(3.0)
        await drone.action.disarm()
        await asyncio.sleep(1.5)
    except Exception:
        pass


async def run_baseline_episode(drone, buf, max_steps, start_n, start_e, ep_num):
    alts  = []
    dists = []
    ok, actual_pos_d = await arm_and_climb(drone, buf, start_n, start_e)
    if not ok:
        await land_and_disarm(drone)
        return None
    print(f"  [Baseline {ep_num:2d}] At {buf.altitude():.3f}m | Collecting...",
          end=" ", flush=True)
    for _ in range(max_steps):
        await drone.offboard.set_position_ned(
            PositionNedYaw(start_n, start_e, actual_pos_d, 0.0)
        )
        alts.append(buf.altitude())
        dists.append((buf.pos_n**2 + buf.pos_e**2)**0.5)
        await asyncio.sleep(0.05)
    print(f"Done  alt={np.mean(alts):.3f}±{np.std(alts):.3f}m")
    await land_and_disarm(drone)
    return {'altitudes': np.array(alts), 'distances': np.array(dists),
            'start_n': start_n, 'start_e': start_e}


async def run_policy_episode(drone, buf, model, max_steps, start_n, start_e, ep_num):
    alts    = []
    dists   = []
    actions = []
    success = True
    ok, _ = await arm_and_climb(drone, buf, start_n, start_e)
    if not ok:
        await land_and_disarm(drone)
        return None
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(0.1)
    print(f"  [Policy  {ep_num:2d}] At {buf.altitude():.3f}m | Collecting...",
          end=" ", flush=True)
    for _ in range(max_steps):
        obs    = buf.get_obs()
        action = model.predict(obs)
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(float(action[0]), float(action[1]),
                           float(action[2]), 0.0)
        )
        alt  = buf.altitude()
        dist = (buf.pos_n**2 + buf.pos_e**2)**0.5
        alts.append(alt)
        dists.append(dist)
        actions.append(action.copy())
        if alt < 0.2 or alt > 2.5 or dist > 5.0:
            success = False
            break
        await asyncio.sleep(0.05)
    print(f"{'Done' if success else 'ABORT'}  "
          f"alt={np.mean(alts):.3f}±{np.std(alts):.3f}m")
    await land_and_disarm(drone)
    return {'altitudes': np.array(alts), 'distances': np.array(dists),
            'actions': np.array(actions), 'success': success,
            'start_n': start_n, 'start_e': start_e}


def compute_stats(episodes, label):
    valid    = [e for e in episodes if e is not None]
    n        = len(valid)
    if n == 0:
        return {}
    all_alts  = np.concatenate([e['altitudes'] for e in valid])
    ep_means  = np.array([np.mean(e['altitudes']) for e in valid])
    ep_stds   = np.array([np.std(e['altitudes'])  for e in valid])
    ep_rmse   = np.array([
        np.sqrt(np.mean((e['altitudes'] - TARGET_ALT_M)**2)) for e in valid
    ])
    ep_dmeans = np.array([np.mean(e['distances']) for e in valid])
    ep_dmax   = np.array([np.max(e['distances'])  for e in valid])
    ep_drift  = []
    for e in valid:
        steps = np.arange(len(e['altitudes']))
        slope = np.polyfit(steps, e['altitudes'], 1)[0] if len(steps) > 1 else 0.0
        ep_drift.append(slope * 20.0)
    ep_drift = np.array(ep_drift)

    stats = {
        'label': label, 'n_episodes': n, 'target_alt': TARGET_ALT_M,
        'alt_mean':       float(np.mean(ep_means)),
        'alt_error_mean': float(np.mean(np.abs(ep_means - TARGET_ALT_M))),
        'alt_rmse_mean':  float(np.mean(ep_rmse)),
        'alt_rmse_std':   float(np.std(ep_rmse)),
        'alt_std_mean':   float(np.mean(ep_stds)),
        'dist_mean':      float(np.mean(ep_dmeans)),
        'dist_max_abs':   float(np.max(ep_dmax)),
        'drift_rate_mean':float(np.mean(np.abs(ep_drift))),
        'success_rate':   100.0 * sum(1 for e in valid if e.get('success', True)) / n,
        'ep_means':       ep_means.tolist(),
        'ep_stds':        ep_stds.tolist(),
        'ep_rmse':        ep_rmse.tolist(),
        'ep_errors':      np.abs(ep_means - TARGET_ALT_M).tolist(),
        'ep_drift_rates': ep_drift.tolist(),
        'ep_dist_means':  ep_dmeans.tolist(),
        'ep_dist_max':    ep_dmax.tolist(),
        'all_altitudes':  all_alts.tolist(),
        'episode_altitudes': [e['altitudes'].tolist() for e in valid],
        'episode_distances': [e['distances'].tolist() for e in valid],
    }

    if 'actions' in valid[0]:
        all_actions = np.concatenate([e['actions'] for e in valid])
        stats['action_mean']    = np.mean(all_actions, axis=0).tolist()
        stats['action_std']     = np.std(all_actions,  axis=0).tolist()
        stats['action_max_abs'] = np.max(np.abs(all_actions), axis=0).tolist()
        stats['all_actions']    = all_actions.tolist()
        BINS = 40
        hist = {}
        for ax_i, ax_name in enumerate(['vn', 've', 'vd']):
            vals = all_actions[:, ax_i]
            counts, edges = np.histogram(vals, bins=BINS)
            centres = (edges[:-1] + edges[1:]) / 2
            hist[ax_name] = {
                'x': centres.tolist(), 'y': counts.tolist(),
                'edges': edges.tolist(), 'width': float(edges[1]-edges[0]),
                'mean': float(np.mean(vals)), 'std': float(np.std(vals)),
                'median': float(np.median(vals)),
            }
        stats['action_histogram'] = hist
    return stats


def compute_comparison(b, p):
    return {
        'alt_rmse_gap':       p['alt_rmse_mean'] - b['alt_rmse_mean'],
        'alt_std_gap':        p['alt_std_mean']  - b['alt_std_mean'],
        'rmse_ratio':         p['alt_rmse_mean'] / max(b['alt_rmse_mean'], 1e-6),
        'transfer_efficiency': max(0.0, 100.0 * (
            1.0 - (p['alt_rmse_mean'] - b['alt_rmse_mean'])
            / max(b['alt_rmse_mean'], 1e-6)
        )),
    }


def format_report(b, p, c):
    n = b['n_episodes']
    lines = [
        "=" * 72,
        "STAGE 1 STATISTICS — INDOOR (MTF-01 OPTICAL FLOW)",
        f"Target altitude: {TARGET_ALT_M}m | Episodes: {n}",
        "=" * 72, "",
        "TABLE 1 — MAIN COMPARISON",
        "-" * 72,
        f"{'Metric':<35} {'PX4 Baseline':>14} {'BC Policy':>12} {'Gap':>10}",
        "-" * 72,
        f"{'Mean altitude (m)':<35} {b['alt_mean']:>14.3f} {p['alt_mean']:>12.3f} {p['alt_mean']-b['alt_mean']:>+10.3f}",
        f"{'Mean altitude error (m)':<35} {b['alt_error_mean']:>14.3f} {p['alt_error_mean']:>12.3f} {p['alt_error_mean']-b['alt_error_mean']:>+10.3f}",
        f"{'RMSE from target (m)':<35} {b['alt_rmse_mean']:>14.3f} {p['alt_rmse_mean']:>12.3f} {c['alt_rmse_gap']:>+10.3f}",
        f"{'Altitude std dev (m)':<35} {b['alt_std_mean']:>14.3f} {p['alt_std_mean']:>12.3f} {c['alt_std_gap']:>+10.3f}",
        f"{'Mean horiz distance (m)':<35} {b['dist_mean']:>14.3f} {p['dist_mean']:>12.3f} {p['dist_mean']-b['dist_mean']:>+10.3f}",
        f"{'Drift rate |m/s| mean':<35} {b['drift_rate_mean']:>14.4f} {p['drift_rate_mean']:>12.4f} {p['drift_rate_mean']-b['drift_rate_mean']:>+10.4f}",
        f"{'Success rate (%)':<35} {'100.0':>14} {p['success_rate']:>12.1f} {'—':>10}",
        f"{'RMSE ratio (policy/baseline)':<35} {'1.00':>14} {c['rmse_ratio']:>12.2f} {'—':>10}",
        f"{'Transfer efficiency (%)':<35} {'—':>14} {c['transfer_efficiency']:>12.1f} {'—':>10}",
        "-" * 72, "",
        "TABLE 2 — PER-EPISODE BREAKDOWN",
        "-" * 96,
        f"{'Ep':<4} {'BL alt':>8} {'BL std':>7} {'BL RMSE':>8} {'BL dist':>8} {'BC alt':>8} {'BC std':>7} {'BC RMSE':>8} {'BC dist':>8} {'BC drift':>9}",
        "-" * 96,
    ]
    for i in range(min(len(b['ep_means']), len(p['ep_means']))):
        lines.append(
            f"{i+1:<4} {b['ep_means'][i]:>8.3f} {b['ep_stds'][i]:>7.4f} "
            f"{b['ep_rmse'][i]:>8.4f} {b['ep_dist_means'][i]:>8.3f} "
            f"{p['ep_means'][i]:>8.3f} {p['ep_stds'][i]:>7.4f} "
            f"{p['ep_rmse'][i]:>8.4f} {p['ep_dist_means'][i]:>8.3f} "
            f"{p['ep_drift_rates'][i]:>+9.4f}"
        )
    lines += [
        "-" * 96,
        f"{'Mean':<4} {np.mean(b['ep_means']):>8.3f} {np.mean(b['ep_stds']):>7.4f} "
        f"{np.mean(b['ep_rmse']):>8.4f} {np.mean(b['ep_dist_means']):>8.3f} "
        f"{np.mean(p['ep_means']):>8.3f} {np.mean(p['ep_stds']):>7.4f} "
        f"{np.mean(p['ep_rmse']):>8.4f} {np.mean(p['ep_dist_means']):>8.3f} "
        f"{np.mean(np.abs(p['ep_drift_rates'])):>9.4f}",
        "",
        "ACTION DISTRIBUTION",
        "-" * 72,
    ]
    if 'action_mean' in p:
        am, as_, ax = p['action_mean'], p['action_std'], p['action_max_abs']
        lines += [
            f"  vn: mean={am[0]:+.4f}  std={as_[0]:.4f}  max={ax[0]:.4f}",
            f"  ve: mean={am[1]:+.4f}  std={as_[1]:.4f}  max={ax[1]:.4f}",
            f"  vd: mean={am[2]:+.4f}  std={as_[2]:.4f}  max={ax[2]:.4f}",
        ]
    lines += ["", "=" * 72, "All raw data: ./results/indoor/statistics_summary_indoor.pkl", "=" * 72]
    return "\n".join(lines)


async def run_statistics(model_path, num_episodes, max_steps,
                         system_address, results_dir):
    Path(results_dir).mkdir(parents=True, exist_ok=True)
    print(f"\nSTAGE 1 INDOOR STATISTICS — target {TARGET_ALT_M}m")
    print(f"Model: {model_path} | Episodes: {num_episodes} | Steps: {max_steps}\n")

    np.random.seed(42)
    starts = [(float(np.random.uniform(-0.5, 0.5)),
               float(np.random.uniform(-0.5, 0.5)))
              for _ in range(num_episodes)]

    model = HoverPolicy(state_dim=13, action_dim=3)
    model.load_state_dict(torch.load(model_path, map_location='cpu'))
    model.eval()

    drone, buf = await connect_drone(system_address)

    print("CONDITION A — PX4 Baseline")
    baseline_raw = []
    for i, (sn, se) in enumerate(starts):
        r = await run_baseline_episode(drone, buf, max_steps, sn, se, i+1)
        baseline_raw.append(r)

    print("\nCONDITION B — Imitation Policy")
    policy_raw = []
    for i, (sn, se) in enumerate(starts):
        r = await run_policy_episode(drone, buf, model, max_steps, sn, se, i+1)
        policy_raw.append(r)

    b = compute_stats(baseline_raw, "PX4 Optical Flow Hold")
    p = compute_stats(policy_raw,   "Imitation Policy (BC Indoor)")
    c = compute_comparison(b, p)

    summary = {'baseline': b, 'policy': p, 'comparison': c,
                'target_alt': TARGET_ALT_M, 'starts': starts}
    pkl_path = f"{results_dir}/statistics_summary_indoor.pkl"
    with open(pkl_path, 'wb') as f:
        pickle.dump(summary, f)

    report = format_report(b, p, c)
    txt_path = f"{results_dir}/statistics_report_indoor.txt"
    with open(txt_path, 'w') as f:
        f.write(report)

    print("\n" + report)
    print(f"\nSaved: {pkl_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',    type=str, default='./models/hover_policy_indoor_best.pth')
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--steps',    type=int, default=500)
    parser.add_argument('--address',  type=str, default='udp://:14550')
    parser.add_argument('--results',  type=str, default='./results/indoor')
    args = parser.parse_args()

    asyncio.run(run_statistics(
        model_path=args.model,
        num_episodes=args.episodes,
        max_steps=args.steps,
        system_address=args.address,
        results_dir=args.results,
    ))
    os._exit(0)
