"""
STAGE 1 STATISTICS — COMPLETE PAPER DATA EXTRACTOR
====================================================
Runs two conditions back-to-back using identical starting positions:

  Condition A — PX4 Position Hold (Baseline / Expert)
  Condition B — Imitation Policy (Neural Network)

Produces everything needed for the paper:
  - Comparison table (copy directly into paper)
  - Per-episode raw data
  - Action distribution data
  - Time series data for figures
  - All saved to ./results/

Usage:
  python stage1_statistics.py --model ./models/hover_policy_best.pth --episodes 10
  python stage1_statistics.py --model ./models/hover_policy_best.pth --episodes 10 --steps 500

  for SIIL and achieve good data. I'm using this
  python stage_statistics.py  --model ./models/hover_policy_best.pth --episodes 10  --steps 500

  for HITL
  python stage1_statistics.py \
    --model ./models/hover_policy_best.pth \
    --episodes 10 \
    --steps 500 \
    --address serial:///dev/ttyACM0:921600 \
    --results ./results/hitl
    
  or you run run directly without address serial because we'll use the proxy
  python3 stage_statistics.py --model ./models/hover_policy_best.pth --episodes 10 --steps 500 --results ./results/hitl/
  
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


# ─────────────────────────────────────────────
# Policy model
# ─────────────────────────────────────────────
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


# ─────────────────────────────────────────────
# Drone setup
# ─────────────────────────────────────────────
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
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("  Position OK")
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


async def arm_and_climb(drone, buf, target_alt, start_n, start_e):
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
            PositionNedYaw(start_n, start_e, -target_alt, 0.0)
        )
        dd = buf.pos_d - (-target_alt)
        if abs(dd) <= 0.4:
            break
        if asyncio.get_event_loop().time() - t0 > 40.0:
            break
        await asyncio.sleep(0.05)
    # Settle and capture actual hover position
    t1 = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - t1 < 1.5:
        await drone.offboard.set_position_ned(
            PositionNedYaw(start_n, start_e, -target_alt, 0.0)
        )
        await asyncio.sleep(0.05)
    actual_pos_d = buf.pos_d   # actual settled altitude
    return True, actual_pos_d


async def land_and_disarm(drone):
    try:
        await drone.offboard.stop()
        await asyncio.sleep(0.5)
    except Exception:
        pass
    try:
        await drone.action.land()
        await asyncio.sleep(5.0)
        await drone.action.disarm()
        await asyncio.sleep(2.0)
    except Exception:
        pass


# ─────────────────────────────────────────────
# Condition A: PX4 baseline
# ─────────────────────────────────────────────
async def run_baseline_episode(drone, buf, max_steps, target_alt,
                                start_n, start_e, ep_num):
    alts   = []
    dists  = []

    ok, actual_pos_d = await arm_and_climb(
        drone, buf, target_alt, start_n, start_e
    )
    if not ok:
        await land_and_disarm(drone)
        return None

    print(f"  [Baseline {ep_num:2d}] At {buf.altitude():.2f}m | Collecting...",
          end=" ", flush=True)

    for _ in range(max_steps):
        await drone.offboard.set_position_ned(
            PositionNedYaw(start_n, start_e, actual_pos_d, 0.0)
        )
        alts.append(buf.altitude())
        dists.append((buf.pos_n**2 + buf.pos_e**2)**0.5)
        await asyncio.sleep(0.05)

    print(f"Done  alt={np.mean(alts):.3f}±{np.std(alts):.3f}m  "
          f"dist={np.mean(dists):.3f}m")
    await land_and_disarm(drone)

    return {
        'altitudes':  np.array(alts),
        'distances':  np.array(dists),
        'start_n':    start_n,
        'start_e':    start_e,
        'start_dist': (start_n**2 + start_e**2)**0.5
    }


# ─────────────────────────────────────────────
# Condition B: Imitation policy
# ─────────────────────────────────────────────
async def run_policy_episode(drone, buf, model, max_steps, target_alt,
                              start_n, start_e, ep_num):
    alts    = []
    dists   = []
    actions = []
    success = True

    ok, actual_pos_d = await arm_and_climb(
        drone, buf, target_alt, start_n, start_e
    )
    if not ok:
        await land_and_disarm(drone)
        return None

    # Switch to velocity mode
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(0.1)

    print(f"  [Policy  {ep_num:2d}] At {buf.altitude():.2f}m | Collecting...",
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
        if alt < 2.0 or alt > 25.0 or dist > 20.0:
            success = False
            break
        await asyncio.sleep(0.05)

    print(f"{'Done' if success else 'ABORT'}  "
          f"alt={np.mean(alts):.3f}±{np.std(alts):.3f}m  "
          f"dist={np.mean(dists):.3f}m")
    await land_and_disarm(drone)

    return {
        'altitudes':  np.array(alts),
        'distances':  np.array(dists),
        'actions':    np.array(actions),
        'success':    success,
        'start_n':    start_n,
        'start_e':    start_e,
        'start_dist': (start_n**2 + start_e**2)**0.5
    }


# ─────────────────────────────────────────────
# Statistics computation
# ─────────────────────────────────────────────
def compute_stats(episodes, target_alt, label):
    valid = [e for e in episodes if e is not None]
    n     = len(valid)
    if n == 0:
        return {}

    all_alts  = np.concatenate([e['altitudes'] for e in valid])
    all_dists = np.concatenate([e['distances'] for e in valid])

    ep_means  = np.array([np.mean(e['altitudes'])  for e in valid])
    ep_stds   = np.array([np.std(e['altitudes'])   for e in valid])
    ep_rmse   = np.array([
        np.sqrt(np.mean((e['altitudes'] - target_alt)**2)) for e in valid
    ])
    ep_errors = np.abs(ep_means - target_alt)
    ep_dmeans = np.array([np.mean(e['distances'])  for e in valid])
    ep_dmax   = np.array([np.max(e['distances'])   for e in valid])

    # Drift rate: linear regression of altitude vs step index
    ep_drift_rates = []
    for e in valid:
        steps = np.arange(len(e['altitudes']))
        if len(steps) > 1:
            slope = np.polyfit(steps, e['altitudes'], 1)[0]
            ep_drift_rates.append(slope * 20.0)  # m/s (20Hz × slope per step)
        else:
            ep_drift_rates.append(0.0)
    ep_drift_rates = np.array(ep_drift_rates)

    stats = {
        'label':            label,
        'n_episodes':       n,
        'target_alt':       target_alt,

        # Altitude accuracy
        'alt_mean':         float(np.mean(ep_means)),
        'alt_error_mean':   float(np.mean(ep_errors)),
        'alt_error_std':    float(np.std(ep_errors)),
        'alt_error_max':    float(np.max(ep_errors)),
        'alt_rmse_mean':    float(np.mean(ep_rmse)),
        'alt_rmse_std':     float(np.std(ep_rmse)),

        # Altitude stability
        'alt_std_mean':     float(np.mean(ep_stds)),
        'alt_std_std':      float(np.std(ep_stds)),
        'alt_global_std':   float(np.std(all_alts)),

        # Drift rate
        'drift_rate_mean':  float(np.mean(np.abs(ep_drift_rates))),
        'drift_rate_std':   float(np.std(ep_drift_rates)),

        # Horizontal position
        'dist_mean':        float(np.mean(ep_dmeans)),
        'dist_max_mean':    float(np.mean(ep_dmax)),
        'dist_max_abs':     float(np.max(ep_dmax)),

        # Success
        'success_rate':     100.0 * sum(
            1 for e in valid if e.get('success', True)
        ) / n,

        # Per-episode arrays for figures
        'ep_means':         ep_means.tolist(),
        'ep_stds':          ep_stds.tolist(),
        'ep_errors':        ep_errors.tolist(),
        'ep_rmse':          ep_rmse.tolist(),
        'ep_drift_rates':   ep_drift_rates.tolist(),
        'ep_dist_means':    ep_dmeans.tolist(),
        'ep_dist_max':      ep_dmax.tolist(),

        # Full time series (for altitude plot figure)
        'all_altitudes':    all_alts.tolist(),
        'episode_altitudes': [e['altitudes'].tolist() for e in valid],
        'episode_distances': [e['distances'].tolist() for e in valid],
    }

    # Action distribution (policy only)
    if 'actions' in valid[0]:
        all_actions = np.concatenate([e['actions'] for e in valid])
        stats['action_mean']    = np.mean(all_actions, axis=0).tolist()
        stats['action_std']     = np.std(all_actions, axis=0).tolist()
        stats['action_max_abs'] = np.max(np.abs(all_actions), axis=0).tolist()
        stats['all_actions']    = all_actions.tolist()

        # Pre-computed histogram x/y data for each axis — ready for direct plotting
        # x = bin_centres (m/s),  y = counts (number of steps)
        # Usage: plt.bar(hist['vd']['x'], hist['vd']['y'], width=hist['vd']['width'])
        BINS = 40
        hist = {}
        axis_names = ['vn', 've', 'vd']
        for ax_i, ax_name in enumerate(axis_names):
            vals              = all_actions[:, ax_i]
            counts, bin_edges = np.histogram(vals, bins=BINS)
            bin_centres       = ((bin_edges[:-1] + bin_edges[1:]) / 2)
            bin_width         = float(bin_edges[1] - bin_edges[0])
            hist[ax_name] = {
                'x':      bin_centres.tolist(),   # bin centre values (m/s)
                'y':      counts.tolist(),         # frequency (step count per bin)
                'edges':  bin_edges.tolist(),      # raw bin edges if needed
                'width':  bin_width,               # bar width for plt.bar()
                'mean':   float(np.mean(vals)),
                'std':    float(np.std(vals)),
                'median': float(np.median(vals)),
            }
        stats['action_histogram'] = hist

    return stats


def compute_comparison(b, p):
    """Transfer metrics — how well policy matches baseline."""
    return {
        'alt_error_gap':        p['alt_error_mean']  - b['alt_error_mean'],
        'alt_rmse_gap':         p['alt_rmse_mean']   - b['alt_rmse_mean'],
        'alt_std_gap':          p['alt_std_mean']    - b['alt_std_mean'],
        'dist_gap':             p['dist_mean']       - b['dist_mean'],
        'drift_rate_gap':       p['drift_rate_mean'] - b['drift_rate_mean'],

        # Ratio: 1.0 = perfect match, higher = worse
        'rmse_ratio':           p['alt_rmse_mean'] / max(b['alt_rmse_mean'], 1e-6),
        'std_ratio':            p['alt_std_mean']  / max(b['alt_std_mean'],  1e-6),

        # Transfer efficiency: 100% = policy matches baseline perfectly
        'transfer_efficiency':  max(0.0, 100.0 * (
            1.0 - (p['alt_rmse_mean'] - b['alt_rmse_mean'])
            / max(b['alt_rmse_mean'], 1e-6)
        )),
    }


def format_report(b, p, c, target_alt):
    n_ep = b['n_episodes']
    lines = [
        "=" * 72,
        "STAGE 1 STATISTICS REPORT",
        f"Behavioural Cloning: PX4 Position Hold vs Imitation Policy",
        f"Target altitude: {target_alt}m | Episodes: {n_ep} | Steps: per episode",
        "=" * 72,
        "",
        # ── Table 1: Main comparison ──
        "TABLE 1 — MAIN COMPARISON (copy directly to paper)",
        "-" * 72,
        f"{'Metric':<35} {'PX4 Baseline':>14} {'BC Policy':>12} {'Gap':>10}",
        "-" * 72,
        f"{'Mean altitude (m)':<35} {b['alt_mean']:>14.3f} {p['alt_mean']:>12.3f} "
        f"{p['alt_mean']-b['alt_mean']:>+10.3f}",
        f"{'Mean altitude error (m)':<35} {b['alt_error_mean']:>14.3f} "
        f"{p['alt_error_mean']:>12.3f} {c['alt_error_gap']:>+10.3f}",
        f"{'RMSE from target (m)':<35} {b['alt_rmse_mean']:>14.3f} "
        f"{p['alt_rmse_mean']:>12.3f} {c['alt_rmse_gap']:>+10.3f}",
        f"{'Altitude std dev (m)':<35} {b['alt_std_mean']:>14.3f} "
        f"{p['alt_std_mean']:>12.3f} {c['alt_std_gap']:>+10.3f}",
        f"{'Mean horiz distance (m)':<35} {b['dist_mean']:>14.3f} "
        f"{p['dist_mean']:>12.3f} {c['dist_gap']:>+10.3f}",
        f"{'Max horiz distance (m)':<35} {b['dist_max_abs']:>14.3f} "
        f"{p['dist_max_abs']:>12.3f} {'—':>10}",
        f"{'Drift rate |m/s| mean':<35} {b['drift_rate_mean']:>14.4f} "
        f"{p['drift_rate_mean']:>12.4f} {c['drift_rate_gap']:>+10.4f}",
        f"{'Success rate (%)':<35} {'100.0':>14} {p['success_rate']:>12.1f} {'—':>10}",
        f"{'RMSE ratio (policy/baseline)':<35} {'1.00':>14} {c['rmse_ratio']:>12.2f} {'—':>10}",
        f"{'Transfer efficiency (%)':<35} {'—':>14} {c['transfer_efficiency']:>12.1f} {'—':>10}",
        "-" * 72,
        "",
        # ── Table 2: Per-episode ──
        "TABLE 2 — PER-EPISODE BREAKDOWN",
        "-" * 104,
        f"{'Ep':<4} {'BL alt':>8} {'BL std':>7} {'BL RMSE':>8} {'BL dist':>8} {'BL dmax':>8} "
        f"{'BC alt':>8} {'BC std':>7} {'BC RMSE':>8} {'BC dist':>8} {'BC dmax':>8} {'BC drift':>9}",
        "-" * 104,
    ]

    n = min(len(b['ep_means']), len(p['ep_means']))
    for i in range(n):
        lines.append(
            f"{i+1:<4} {b['ep_means'][i]:>8.3f} {b['ep_stds'][i]:>7.4f} "
            f"{b['ep_rmse'][i]:>8.4f} {b['ep_dist_means'][i]:>8.3f} {b['ep_dist_max'][i]:>8.3f} "
            f"{p['ep_means'][i]:>8.3f} {p['ep_stds'][i]:>7.4f} "
            f"{p['ep_rmse'][i]:>8.4f} {p['ep_dist_means'][i]:>8.3f} {p['ep_dist_max'][i]:>8.3f} "
            f"{p['ep_drift_rates'][i]:>+9.4f}"
        )

    lines += [
        "-" * 104,
        f"{'Mean':<4} {np.mean(b['ep_means']):>8.3f} {np.mean(b['ep_stds']):>7.4f} "
        f"{np.mean(b['ep_rmse']):>8.4f} {np.mean(b['ep_dist_means']):>8.3f} {np.mean(b['ep_dist_max']):>8.3f} "
        f"{np.mean(p['ep_means']):>8.3f} {np.mean(p['ep_stds']):>7.4f} "
        f"{np.mean(p['ep_rmse']):>8.4f} {np.mean(p['ep_dist_means']):>8.3f} {np.mean(p['ep_dist_max']):>8.3f} "
        f"{np.mean(np.abs(p['ep_drift_rates'])):>9.4f}",
        "",
        # ── Action distribution ──
        "ACTION DISTRIBUTION (policy output statistics)",
        "-" * 72,
    ]

    if 'action_mean' in p:
        am = p['action_mean']
        as_ = p['action_std']
        ax = p['action_max_abs']
        lines += [
            f"  vn (north): mean={am[0]:+.4f}  std={as_[0]:.4f}  max|val|={ax[0]:.4f}",
            f"  ve (east):  mean={am[1]:+.4f}  std={as_[1]:.4f}  max|val|={ax[1]:.4f}",
            f"  vd (down):  mean={am[2]:+.4f}  std={as_[2]:.4f}  max|val|={ax[2]:.4f}",
            "",
            "  NOTE: Near-zero means confirm unbiased labels (TARGET_D fix working).",
            "  Small std confirms policy outputs gentle corrections, not aggressive cmds.",
        ]

    lines += [
        "",
        "=" * 72,
        "ACTION HISTOGRAM — X/Y DATA FOR PLOTTING",
        "-" * 72,
    ]

    if 'action_histogram' in p:
        h = p['action_histogram']
        for ax in ['vn', 've', 'vd']:
            lines += [
                f"  {ax}:  mean={h[ax]['mean']:+.4f}  std={h[ax]['std']:.4f}  "
                f"median={h[ax]['median']:+.4f}  bins={len(h[ax]['x'])}",
                f"       x (bin centres): {[round(v,4) for v in h[ax]['x'][:5]]} ... "
                f"{[round(v,4) for v in h[ax]['x'][-5:]]}",
                f"       y (counts):      {h[ax]['y'][:5]} ... {h[ax]['y'][-5:]}",
                "",
            ]
        lines += [
            "  To plot in Python:",
            "    import pickle, numpy as np, matplotlib.pyplot as plt",
            "    with open('./results/statistics_summary.pkl','rb') as f: d=pickle.load(f)",
            "    h = d['policy']['action_histogram']['vd']",
            "    plt.bar(h['x'], h['y'], width=h['width'], color='steelblue')",
            "    plt.axvline(h['mean'], color='orange', label=f\"mean={h['mean']:.4f}\")",
            "    plt.axvline(0, color='red', linestyle='--', label='zero')",
            "    plt.xlabel('vd command (m/s)'); plt.ylabel('frequency')",
            "    plt.legend(); plt.show()",
        ]

    lines += [
        "",
        "=" * 72,
        "FIGURES — ALL DATA KEYS IN statistics_summary.pkl",
        "-" * 72,
        "  Fig 1 — Altitude time series:",
        "    data['baseline']['episode_altitudes']  → list of 10 lists (500 steps each)",
        "    data['policy']['episode_altitudes']    → list of 10 lists (500 steps each)",
        "",
        "  Fig 2 — Per-episode RMSE bar chart:",
        "    data['baseline']['ep_rmse']            → list of 10 RMSE values",
        "    data['policy']['ep_rmse']              → list of 10 RMSE values",
        "",
        "  Fig 3 — Action distribution histogram (vd):",
        "    data['policy']['action_histogram']['vd']['x']  → bin centres (m/s)",
        "    data['policy']['action_histogram']['vd']['y']  → counts per bin",
        "    data['policy']['action_histogram']['vd']['width'] → bar width",
        "    data['policy']['all_actions']          → raw (N,3) array [vn,ve,vd]",
        "",
        "  Fig 4 — Horizontal drift bar chart:",
        "    data['baseline']['ep_dist_max']        → max dist per episode",
        "    data['policy']['ep_dist_max']          → max dist per episode",
        "",
        "  Extra — Horizontal distance time series:",
        "    data['baseline']['episode_distances']  → list of 10 lists (500 steps each)",
        "    data['policy']['episode_distances']    → list of 10 lists (500 steps each)",
        "",
        "  All raw data: ./results/statistics_summary.pkl",
        "=" * 72,
    ]

    return "\n".join(lines)


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────
async def run_statistics(model_path, num_episodes, max_steps,
                         target_alt, system_address, results_dir):

    Path(results_dir).mkdir(parents=True, exist_ok=True)

    print("\n" + "=" * 72)
    print("STAGE 1 STATISTICS: PX4 BASELINE vs IMITATION POLICY")
    print("=" * 72)
    print(f"Model:       {model_path}")
    print(f"Episodes:    {num_episodes} per condition  ({num_episodes*2} total flights)")
    print(f"Steps:       {max_steps} per episode  ({max_steps*0.05:.0f}s each)")
    print(f"Target alt:  {target_alt}m")
    print(f"Est. time:   ~{num_episodes * 2 * 26 / 60:.0f} min")
    print("=" * 72 + "\n")

    # Fixed seed ensures identical start positions for both conditions
    np.random.seed(42)
    starts = [
        (float(np.random.uniform(-2, 2)), float(np.random.uniform(-2, 2)))
        for _ in range(num_episodes)
    ]
    print("Start positions (same for both conditions):")
    for i, (n, e) in enumerate(starts):
        print(f"  Ep {i+1:2d}: N={n:+.2f}m  E={e:+.2f}m  "
              f"dist={np.sqrt(n**2+e**2):.2f}m")
    print()

    print("Loading model...")
    model = HoverPolicy(state_dim=13, action_dim=3)
    model.load_state_dict(torch.load(model_path, map_location='cpu'))
    model.eval()

    print("Connecting to drone...")
    drone, buf = await connect_drone(system_address)

    # ── Condition A ──
    print("=" * 72)
    print("CONDITION A — PX4 Position Hold (Baseline)")
    print("=" * 72)
    baseline_raw = []
    for i, (sn, se) in enumerate(starts):
        r = await run_baseline_episode(
            drone, buf, max_steps, target_alt, sn, se, i+1
        )
        baseline_raw.append(r)

    # ── Condition B ──
    print("\n" + "=" * 72)
    print("CONDITION B — Imitation Policy (Neural Network)")
    print("=" * 72)
    policy_raw = []
    for i, (sn, se) in enumerate(starts):
        r = await run_policy_episode(
            drone, buf, model, max_steps, target_alt, sn, se, i+1
        )
        policy_raw.append(r)

    # ── Compute ──
    print("\nComputing statistics...")
    b = compute_stats(baseline_raw, target_alt, "PX4 Position Hold")
    p = compute_stats(policy_raw,   target_alt, "Imitation Policy (BC)")
    c = compute_comparison(b, p)

    # ── Save ──
    summary = {
        'baseline': b, 'policy': p, 'comparison': c,
        'target_alt': target_alt, 'num_episodes': num_episodes,
        'max_steps': max_steps, 'starts': starts,
    }
    with open(f"{results_dir}/statistics_summary.pkl", 'wb') as f:
        pickle.dump(summary, f)

    report = format_report(b, p, c, target_alt)
    report_path = f"{results_dir}/statistics_report.txt"
    with open(report_path, 'w') as f:
        f.write(report)

    print("\n" + report)
    print(f"\nSaved to {results_dir}/")
    print(f"  statistics_summary.pkl  — all raw data for figures")
    print(f"  statistics_report.txt   — paper tables")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',    type=str, default='./models/hover_policy_best.pth')
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--steps',    type=int, default=200)
    parser.add_argument('--altitude', type=float, default=10.0)
    parser.add_argument('--address',  type=str, default='udp://:14550')
    parser.add_argument('--results',  type=str, default='./results')
    args = parser.parse_args()

    asyncio.run(run_statistics(
        model_path=args.model,
        num_episodes=args.episodes,
        max_steps=args.steps,
        target_alt=args.altitude,
        system_address=args.address,
        results_dir=args.results,
    ))
    os._exit(0)
