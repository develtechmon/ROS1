"""
Print baseline RMSE from existing statistics_summary.pkl
Run: python print_baseline_rmse.py
"""
import pickle
import numpy as np

with open('./results/statistics_summary.pkl', 'rb') as f:
    data = pickle.load(f)

b = data['baseline']
p = data['policy']

print("\nTABLE 2 — WITH BASELINE RMSE (updated)")
print("-" * 82)
print(f"{'Ep':<4} {'BL alt':>8} {'BL std':>7} {'BL RMSE':>8} "
      f"{'BC alt':>8} {'BC std':>7} {'BC RMSE':>8}")
print("-" * 82)

for i in range(len(b['ep_means'])):
    print(f"{i+1:<4} {b['ep_means'][i]:>8.3f} {b['ep_stds'][i]:>7.4f} "
          f"{b['ep_rmse'][i]:>8.4f} "
          f"{p['ep_means'][i]:>8.3f} {p['ep_stds'][i]:>7.4f} "
          f"{p['ep_rmse'][i]:>8.4f}")

print("-" * 82)
print(f"{'Mean':<4} {np.mean(b['ep_means']):>8.3f} {np.mean(b['ep_stds']):>7.4f} "
      f"{np.mean(b['ep_rmse']):>8.4f} "
      f"{np.mean(p['ep_means']):>8.3f} {np.mean(p['ep_stds']):>7.4f} "
      f"{np.mean(p['ep_rmse']):>8.4f}")

print(f"\nRMSE ratio: {np.mean(p['ep_rmse']) / np.mean(b['ep_rmse']):.2f}×")
print(f"\nWhy baseline RMSE is not zero:")
print(f"  PX4 hovers at {np.mean(b['ep_means']):.3f}m, not 10.000m")
print(f"  Barometric offset: {10.0 - np.mean(b['ep_means']):.3f}m below target")
print(f"  This offset alone contributes ~{10.0 - np.mean(b['ep_means']):.3f}m to baseline RMSE")