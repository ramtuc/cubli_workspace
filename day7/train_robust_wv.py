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

import gymnasium as gym
import numpy as np
import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, VecMonitor
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
from cubli_env_wv import CubliEnv
from train_utils import LearningCurveCallback


#class RandomKickWrapper(gym.ActionWrapper):
#    """
#    行動にランダムな外乱トルクを確率的に混入するラッパー。
#    エージェントに「意図通りに体が動かない状況」を経験させ、
#    実機の摩擦・モデル誤差への汎化（ドメインランダマイゼーション）を促す。
#    """
#    def __init__(self, env, kick_prob=0.02, kick_power=2.0):#0.8):
#        super().__init__(env)
#        self.kick_prob  = kick_prob   # キックが発生する確率
#        self.kick_power = kick_power  # 外乱の最大強度（action スケール）→ 実トルク = 0.8 × 0.5 = 0.4 Nm
#
#    def action(self, action):
#        if np.random.rand() < self.kick_prob:
#            kick = np.random.uniform(-self.kick_power, self.kick_power, size=action.shape)
#            return action + kick
#        return action
#  g073_train_robust.py:35 の np.random.rand() はグローバル状態に依存し、並列環境間で干渉する可能性がある：  
class RandomKickWrapper(gym.ActionWrapper):
    def __init__(self, env, kick_prob=0.02, kick_power=0.8):
        super().__init__(env)
        self.kick_prob  = kick_prob
        self.kick_power = kick_power
        self._rng = np.random.default_rng()  # ローカルRNG

    def action(self, action):
        if self._rng.random() < self.kick_prob:
            kick = self._rng.uniform(-self.kick_power, self.kick_power, size=action.shape)
            return action + kick
        return action

def make_env():
    env = CubliEnv(render_mode=None)
    env = RandomKickWrapper(env, kick_prob=0.02, kick_power=0.8)
    return env

# 8並列で環境を構築し学習を高速化
#  8並列の意味
#   SubprocVecEnv を使うと 8 つの環境が別プロセスで同時に動きます。 
#   1 プロセスで 2M ステップ = 約 2.5 時間が、8 並列で約 30 分に短縮されます。 
#   ただし CPU コア数の上限があるため、コア数を超える並列数は逆効果です。
# 学習後は models/PPO_Robust/ に final_model.zip と vec_normalize.pkl が保存されます。
if __name__ == "__main__":
    MODELS_DIR = "models/PPO_Robust_WV"
    LOG_DIR    = "logs/PPO_Robust_WV"
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
    # 12,500 ステップごとに中間モデルを保存
    checkpoint_callback = CheckpointCallback(
        save_freq=100_000 // num_cpu,
        save_path=MODELS_DIR,
        name_prefix="ppo_cubli_robust",
    )
    curve_callback = LearningCurveCallback(log_dir=LOG_DIR, plot_freq=50_000)

    print(">>> Start Training (Robust)...")
    model.learn(total_timesteps=2_000_000,
                callback=CallbackList([checkpoint_callback, curve_callback]))

    # 9. 実装上の注意点
    #  9.1 VecNormalize の保存と復元を忘れない
    # 学習時と推論時で正規化パラメータが異なると、入力スケールが完全に狂い動作しません。
    model.save(f"{MODELS_DIR}/final_model")
    env.save(f"{MODELS_DIR}/vec_normalize.pkl")   # 学習後  # 正規化パラメータも忘れず保存
    print(">>> Done!")
    env.close()
