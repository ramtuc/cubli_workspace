import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, VecMonitor
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
from cubli_env import CubliEnv
from train_utils import LearningCurveCallback


def make_env():
    return CubliEnv(render_mode=None)


if __name__ == "__main__":
    MODELS_DIR = "models/PPO_Normal"
    LOG_DIR    = "logs/PPO_Normal"
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
        device="cpu",
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=100_000 // num_cpu,
        save_path=MODELS_DIR,
        name_prefix="ppo_cubli_normal",
    )
    curve_callback = LearningCurveCallback(log_dir=LOG_DIR, plot_freq=50_000)

    print(">>> Start Training (Normal)...")
    model.learn(total_timesteps=2_000_000,
                callback=CallbackList([checkpoint_callback, curve_callback]))

    model.save(f"{MODELS_DIR}/final_model")
    env.save(f"{MODELS_DIR}/vec_normalize.pkl")
    print(">>> Done!")
    env.close()
