import gymnasium as gym
import numpy as np
import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, VecMonitor
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
from cubli_env import CubliEnv
from train_utils import LearningCurveCallback


class RandomKickWrapper(gym.ActionWrapper):
    """
    行動にランダムな外乱トルクを確率的に混入するラッパー。
    エージェントに「意図通りに体が動かない状況」を経験させ、
    実機の摩擦・モデル誤差への汎化（ドメインランダマイゼーション）を促す。
    """
    def __init__(self, env, kick_prob=0.02, kick_power=0.8):
        super().__init__(env)
        self.kick_prob  = kick_prob   # キックが発生する確率
        self.kick_power = kick_power  # 外乱の最大強度（action スケール）→ 実トルク = 0.8 × 0.5 = 0.4 Nm

    def action(self, action):
        if np.random.rand() < self.kick_prob:
            kick = np.random.uniform(-self.kick_power, self.kick_power, size=action.shape)
            return action + kick
        return action


def make_env():
    env = CubliEnv(render_mode=None)
    env = RandomKickWrapper(env, kick_prob=0.02, kick_power=0.8)
    return env


if __name__ == "__main__":
    MODELS_DIR = "models/PPO_Robust"
    LOG_DIR    = "logs/PPO_Robust"
    os.makedirs(MODELS_DIR, exist_ok=True)
    os.makedirs(LOG_DIR,    exist_ok=True)

    num_cpu = 8
    env = SubprocVecEnv([make_env for _ in range(num_cpu)])
    env = VecMonitor(env, filename=f"{LOG_DIR}/monitor.csv")
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log=LOG_DIR,
        learning_rate=0.0003,
        n_steps=2048,
        batch_size=64,
        gamma=0.99,
        device="cpu",  # GPU があれば "cuda"
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=100_000 // num_cpu,
        save_path=MODELS_DIR,
        name_prefix="ppo_cubli_robust",
    )
    curve_callback = LearningCurveCallback(log_dir=LOG_DIR, plot_freq=50_000)

    print(">>> Start Training (Robust)...")
    model.learn(total_timesteps=2_000_000,
                callback=CallbackList([checkpoint_callback, curve_callback]))

    model.save(f"{MODELS_DIR}/final_model")
    env.save(f"{MODELS_DIR}/vec_normalize.pkl")  # 正規化パラメータも忘れず保存
    print(">>> Done!")
    env.close()
