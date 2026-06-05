"""
How to get the real vd distribution
The statistics script saves all raw data to ./results/statistics_summary.pkl. The actual vd commands for every step of every episode are stored there 
under policy → all_actions. You can extract and plot the real histogram with this script:
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt

with open('./results/statistics_summary.pkl', 'rb') as f:
    data = pickle.load(f)

# Extract all vd commands across all episodes
all_actions = np.array(data['policy']['all_actions'])
vd_commands  = all_actions[:, 2]   # column 2 = vd

print(f"Total commands: {len(vd_commands)}")
print(f"Mean vd:  {np.mean(vd_commands):.4f} m/s")
print(f"Std vd:   {np.std(vd_commands):.4f} m/s")
print(f"Min vd:   {np.min(vd_commands):.4f} m/s")
print(f"Max vd:   {np.max(vd_commands):.4f} m/s")

# Plot real histogram
plt.figure(figsize=(8, 4))
plt.hist(vd_commands, bins=40, color='#378ADD', alpha=0.8, edgecolor='none')
plt.axvline(x=0, color='red', linestyle='--', linewidth=1, label='zero line')
plt.axvline(x=np.mean(vd_commands), color='orange', linestyle='-',
            linewidth=1.5, label=f'mean={np.mean(vd_commands):.4f}')
plt.xlabel('vd command (m/s)')
plt.ylabel('frequency (steps)')
plt.title('Vertical velocity command distribution — BC policy')
plt.legend()
plt.tight_layout()
plt.savefig('./results/vd_distribution.png', dpi=150)
plt.show()
print("Saved: ./results/vd_distribution.png")