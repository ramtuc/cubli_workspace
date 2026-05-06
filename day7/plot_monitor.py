import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# 引数でCSVパスを指定可能、デフォルトは Robust
csv_path = sys.argv[1] if len(sys.argv) > 1 else "logs/PPO_normal/monitor.csv"
window   = int(sys.argv[2]) if len(sys.argv) > 2 else 200

df = pd.read_csv(csv_path, skiprows=1)  # 1行目はメタ情報なのでスキップ
df["survival_s"] = df["l"] / 240.0
df["episode"]    = np.arange(len(df))

r_mean = df["r"].rolling(window, min_periods=1).mean()
s_mean = df["survival_s"].rolling(window, min_periods=1).mean()

label = os.path.basename(os.path.dirname(csv_path))
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
fig.suptitle(f"Learning Curve — {label}  ({len(df)} episodes,  rolling mean {window} ep)")
plt.subplots_adjust(hspace=0.4)

ax1.plot(df["episode"], r_mean, color="steelblue", linewidth=1.5)
ax1.set_title("Episode Reward")
ax1.set_ylabel("Reward")
ax1.axhline(0, color="k", linewidth=0.5)
ax1.grid(True, alpha=0.3)

ax2.plot(df["episode"], s_mean, color="seagreen", linewidth=1.5)
ax2.set_title("Survival Time [s]")
ax2.set_ylabel("seconds")
ax2.set_xlabel("Episode")
ax2.axhline(0, color="k", linewidth=0.5)
ax2.grid(True, alpha=0.3)

out_path = os.path.join(os.path.dirname(csv_path), "learning_curve_smooth.png")
plt.tight_layout()
plt.savefig(out_path, dpi=120)
print(f"保存: {out_path}")
plt.show()
