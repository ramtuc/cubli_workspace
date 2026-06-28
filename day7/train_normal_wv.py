# 7. 実装コード解説
# 今回の実装は以下の 8 ファイルで構成されています。
# ファイル	          役割
# cubli_env.py	     PyBullet + Gymnasium の RL 環境本体。観測・報酬・終了判定を定義
# train_normal.py	 外乱なしで PPO 学習（比較用ベースライン）
# train_robust.py	 Random Kick を加えた PPO 学習（ドメインランダマイゼーション版）
# train_utils.py	 学習曲線を定期保存するコールバック
# enjoy_normal.py	 Normal モデルを GUI で動かして評価・可視化
# enjoy_robust.py	 Robust モデルを GUI で動かして評価・可視化
# compare_models.py	 2 台を並べて同一外乱を与え、両モデルの挙動を比較
# plot_monitor.py	 学習後に monitor.csv から学習曲線グラフを生成

import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, VecMonitor
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
from cubli_env_wv import CubliEnv
from train_utils import LearningCurveCallback

# train_robust.py から RandomKickWrapper を除いただけの、比較用ベースライン学習スクリプトです。
# それ以外の構成（8 並列・2M ステップ・VecNormalize・VecMonitor）は train_robust.py と完全に同一です。
# 学習後は models/PPO_Normal/ に final_model.zip と vec_normalize.pkl が保存されます。
def make_env():
    return CubliEnv(render_mode=None)   # ラッパーなし


if __name__ == "__main__":
    MODELS_DIR = "models/PPO_Normal_WV"
    LOG_DIR    = "logs/PPO_Normal_WV"
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
        name_prefix="ppo_cubli_normal",
    )
    curve_callback = LearningCurveCallback(log_dir=LOG_DIR, plot_freq=50_000)

    print(">>> Start Training (Normal)...")
    model.learn(total_timesteps=2_000_000,
                callback=CallbackList([checkpoint_callback, curve_callback]))

    # 9. 実装上の注意点
    #  9.1 VecNormalize の保存と復元を忘れない
    # 学習時と推論時で正規化パラメータが異なると、入力スケールが完全に狂い動作しません。
    model.save(f"{MODELS_DIR}/final_model")
    env.save(f"{MODELS_DIR}/vec_normalize.pkl")     # 学習後
    print(">>> Done!")
    env.close()



