import numpy as np
import matplotlib
matplotlib.use("Agg")  # ヘッドレス学習用（display不要）
import matplotlib.pyplot as plt
from stable_baselines3.common.callbacks import BaseCallback


class LearningCurveCallback(BaseCallback):
    """
    エピソード報酬と生存時間の学習曲線を定期的にPNGで保存するコールバック。
    """
    def __init__(self, log_dir, plot_freq=50_000, window=200, dt=1.0/240.0, verbose=0):
        super().__init__(verbose)
        self.log_dir   = log_dir
        self.plot_freq = plot_freq
        self.window    = window
        self.dt        = dt
        self._rewards  = []
        self._lengths  = []
        self._steps    = []

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            ep = info.get("episode")
            if ep is not None:
                self._rewards.append(ep["r"])
                self._lengths.append(ep["l"])
                self._steps.append(self.num_timesteps)

        if self.n_calls % self.plot_freq == 0 and self._rewards:
            self._save_plot()
        return True

    def _on_training_end(self):
        if self._rewards:
            self._save_plot()

    def _save_plot(self):
        rewards  = np.array(self._rewards)
        lengths  = np.array(self._lengths)
        steps    = np.array(self._steps)
        survival = lengths * self.dt

        w      = min(self.window, len(rewards))
        kernel = np.ones(w) / w
        r_mean = np.convolve(rewards,  kernel, mode="valid")
        s_mean = np.convolve(survival, kernel, mode="valid")
        ts     = steps[w - 1:]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        fig.suptitle(f"Learning Curve  ({self.num_timesteps:,} steps / {len(rewards)} episodes)")
        plt.subplots_adjust(hspace=0.4)

        ax1.plot(steps, rewards,  alpha=0.2, color="steelblue", linewidth=0.5)
        ax1.plot(ts,    r_mean,   color="steelblue", linewidth=1.5,
                 label=f"Rolling mean ({w} ep)")
        ax1.set_title("Episode Reward")
        ax1.set_ylabel("Reward")
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.3)

        ax2.plot(steps, survival, alpha=0.2, color="seagreen", linewidth=0.5)
        ax2.plot(ts,    s_mean,   color="seagreen", linewidth=1.5,
                 label=f"Rolling mean ({w} ep)")
        ax2.set_title("Survival Time [s]")
        ax2.set_ylabel("seconds")
        ax2.set_xlabel("Timesteps")
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        path = f"{self.log_dir}/learning_curve.png"
        plt.savefig(path, dpi=100)
        plt.close()
        print(f"[LearningCurve] → {path}")
