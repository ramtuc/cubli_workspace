"""
4モデルの学習曲線を1枚に並べて比較する。
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

CONFIGS = [
    ("logs/PPO_Normal/monitor.csv",    "Normal (6-dim)",    "tab:blue",   "--"),
    ("logs/PPO_Robust/monitor.csv",    "Robust (6-dim)",    "tab:red",    "--"),
    ("logs/PPO_Normal_WV/monitor.csv", "Normal+WV (9-dim)", "tab:cyan",   "-"),
    ("logs/PPO_Robust_WV/monitor.csv", "Robust+WV (9-dim)", "tab:orange", "-"),
]
WINDOW = 200

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
fig.suptitle(f"Learning Curve Comparison  (rolling mean {WINDOW} ep)", fontsize=13)
plt.subplots_adjust(hspace=0.45)

for csv_path, label, color, ls in CONFIGS:
    if not os.path.exists(csv_path):
        print(f"スキップ: {csv_path}")
        continue
    df = pd.read_csv(csv_path, skiprows=1)
    df["survival_s"] = df["l"] / 240.0
    df["episode"]    = np.arange(len(df))

    r_mean = df["r"].rolling(WINDOW, min_periods=1).mean()
    s_mean = df["survival_s"].rolling(WINDOW, min_periods=1).mean()

    ax1.plot(df["episode"], r_mean, label=label, color=color, linestyle=ls, linewidth=1.8)
    ax2.plot(df["episode"], s_mean, label=label, color=color, linestyle=ls, linewidth=1.8)
    print(f"{label}: {len(df)} episodes, "
          f"final reward={r_mean.iloc[-1]:.1f}, "
          f"final survival={s_mean.iloc[-1]:.2f}s")

ax1.set_title("Episode Reward")
ax1.set_ylabel("Reward")
ax1.axhline(0, color="k", linewidth=0.5)
ax1.legend(fontsize=9)
ax1.grid(True, alpha=0.3)

ax2.set_title("Survival Time [s]")
ax2.set_ylabel("seconds")
ax2.set_xlabel("Episode")
ax2.axhline(0, color="k", linewidth=0.5)
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3)

out = "day7/comparison_curves.png"
plt.tight_layout()
plt.savefig(out, dpi=130)
print(f"\n保存: {out}")
plt.show()
