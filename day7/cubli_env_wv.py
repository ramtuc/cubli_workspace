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

# from random import seed

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import math
import time

# PyBullet の物理演算と Gymnasium のインターフェースを繋ぐクラスです。
# ① Gymnasium の構造
# 用語：Gymnasium
# 強化学習環境の標準インターフェースです。 
#  reset() でエピソード開始、
#  step(action) で 1 ステップ実行、 
#  close() で終了という構造はすべての環境で共通です。
class CubliEnv(gym.Env):
    def __init__(self, render_mode=None):
        super().__init__()
        self.render_mode = render_mode

        if render_mode == "human":
            self.physicsClient = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(
                cameraDistance=0.4, cameraYaw=45,
                cameraPitch=-15, cameraTargetPosition=[0, 0, 0.1],
                physicsClientId=self.physicsClient,
            )
        else:
            self.physicsClient = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath(),
                                  physicsClientId=self.physicsClient)

        # 行動空間: 3軸トルク [-1.0 ~ 1.0]（実際のトルクは × max_torque）
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        # 観測空間: [角度誤差(3), 角速度(3)]
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32) # 変更後

        self.dt         = 1. / 240.
        self.max_torque = 0.5  # [Nm]

        # 頂点倒立の目標角度（Day 6 と同じ値）
        self.target_roll  = math.pi / 4.0
        self.target_pitch = math.atan(1.0 / math.sqrt(2.0))  # ≈ 35.26°
        self.target_yaw   = 0.0

        self.boxId              = None
        self.episode_error_sum  = 0.0
        self.episode_energy_sum = 0.0
        self.episode_steps      = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        cid = self.physicsClient
        p.resetSimulation(physicsClientId=cid)
        p.setGravity(0, 0, -9.81, physicsClientId=cid)
        if self.render_mode == "human":
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=cid)
        p.loadURDF("plane.urdf", physicsClientId=cid)

        # 初期姿勢にランダムノイズ（ドメインランダマイゼーション）    
        #  4. g071 の seed 管理が Gymnasium 準拠でない
        #  g071_cubli_env.py:74 で reset() に渡された seed を毎回直接 default_rng(seed)
        #    に渡しているため、同じシードを渡すと常に同じノイズが出る（再現性の誤解を招く）。Gymnasium の正しい方法：
        # def reset(self, seed=None, options=None):
        # super().reset(seed=seed)  # self.np_random が更新される
        noise = self.np_random.uniform(-0.1, 0.1, 3)  # self.np_random を使
        #rng   = np.random.default_rng(seed)
        #noise = rng.uniform(-0.1, 0.1, 3)
        start_r = self.target_roll  + noise[0]
        start_p = self.target_pitch + noise[1]
        start_y = self.target_yaw   + noise[2]

        startPos         = [0, 0, 0.15 / math.sqrt(3)]
        startOrientation = p.getQuaternionFromEuler([start_r, start_p, start_y],
                                                    physicsClientId=cid)

        self.boxId = p.loadURDF("day7/my_cubli.urdf", startPos, startOrientation,
                                physicsClientId=cid)
        p.changeDynamics(self.boxId, -1, lateralFriction=1.0, spinningFriction=0.1,
                         physicsClientId=cid)

        for i in range(3):
            p.setJointMotorControl2(self.boxId, i, p.VELOCITY_CONTROL, force=0,
                                    physicsClientId=cid)

        self.episode_error_sum  = 0.0
        self.episode_energy_sum = 0.0
        self.episode_steps      = 0

        return self._get_obs(), {}
    # ③ 報酬計算と終了判定
    def step(self, action):
        cid    = self.physicsClient
        torque = np.array(action) * self.max_torque * -1.0  # ... トルク印加 ...

        p.setJointMotorControl2(self.boxId, 0, p.TORQUE_CONTROL, force=torque[0], physicsClientId=cid)
        p.setJointMotorControl2(self.boxId, 1, p.TORQUE_CONTROL, force=torque[1], physicsClientId=cid)
        p.setJointMotorControl2(self.boxId, 2, p.TORQUE_CONTROL, force=torque[2], physicsClientId=cid)

        p.stepSimulation(physicsClientId=cid)
        if self.render_mode == "human":
            time.sleep(self.dt)

        obs = self._get_obs()
        # err_r, err_p, err_y, vel_r, vel_p, vel_y = obs
        # 変更後
        err_r, err_p, err_y, vel_r, vel_p, vel_y, wvel_x, wvel_y, wvel_z = obs
        # 報酬の内訳：
        # 毎ステップ 1.0 の生存報酬が入りますが、姿勢が崩れるほど大きなペナルティが引かれる構造です。
        #   項	               計算	                                意図
        #   生存報酬            +1.0（固定）	                    倒れずに 1 ステップ耐えるだけで得られる。長く立ち続けるほど累積が増える
        #   角度ペナルティ      10.0 × (err_r² + err_p² + err_y²)	直立からのズレに対してもっとも重いペナルティ。二乗なので傾くほど急激に増大する
        #   角速度ペナルティ	0.1 × (ω_r² + ω_p² + ω_z²)	        姿勢が正しくても激しく揺れている状態を抑制する
        #   トルクペナルティ    0.01 × Σa²	                        不必要な大トルクを抑制し、省エネな動きを促す
        # 完全に直立（誤差ゼロ、角速度ゼロ、トルクゼロ）のとき報酬は最大の 1.0 になります。
        angle_penalty  = 10.0 * (err_r**2 + err_p**2 + err_y**2)
        vel_penalty    =  0.1 * (vel_r**2 + vel_p**2 + vel_y**2)
        action_penalty =  0.01 * np.sum(np.square(action))
        wheel_penalty  =  0.000015 * (wvel_x**2 + wvel_y**2 + wvel_z**2)  # 追加
        reward = 1.0 - angle_penalty - vel_penalty - action_penalty - wheel_penalty
        # reward = 1.0 - angle_penalty - vel_penalty - action_penalty

        self.episode_error_sum  += (abs(err_r) + abs(err_p)) * 180 / math.pi
        self.episode_energy_sum += np.sum(np.abs(torque))
        self.episode_steps      += 1
        # 終了条件：
        #  Roll または Pitch の誤差が 0.8 rad（≈ 46°） を超えると terminated = True になりエピソードが終わります。 
        #  False は truncated（時間切れ）で、今回は時間制限なしのため常に False です。 
        #  {} は追加情報（info）で、転倒時だけエピソード統計が格納されます。
        terminated = bool(abs(err_r) > 0.8 or abs(err_p) > 0.8)
        truncated  = False

        info = {}
        if terminated or truncated:
            info["episode_metrics"] = {
                "avg_angle_error_deg": self.episode_error_sum / max(self.episode_steps, 1),
                "total_energy":        self.episode_energy_sum,
                "survival_time":       self.episode_steps * self.dt,
            }

        return obs, reward, terminated, truncated, info

# ② 観測の取得と正規化
    def _get_obs(self):
        cid = self.physicsClient
        _, quat   = p.getBasePositionAndOrientation(self.boxId, physicsClientId=cid)
        roll, pitch, yaw = p.getEulerFromQuaternion(quat, physicsClientId=cid)
        _, ang_vel = p.getBaseVelocity(self.boxId, physicsClientId=cid)

        err_r = self._wrap(roll  - self.target_roll)    # 目標との差を -π ~ π に正規化
        err_p = self._wrap(pitch - self.target_pitch)   # 目標との差を -π ~ π に正規化
        err_y = self._wrap(yaw   - self.target_yaw)     # 目標との差を -π ~ π に正規化

        # ホイール角速度: joint 0=joint_x(Roll), 1=joint_y(Pitch), 2=joint_z(Yaw)
        wvel_x = p.getJointState(self.boxId, 0, physicsClientId=cid)[1]
        wvel_y = p.getJointState(self.boxId, 1, physicsClientId=cid)[1]
        wvel_z = p.getJointState(self.boxId, 2, physicsClientId=cid)[1]
    #    vel_x = p.getJointState(boxId, joint_map["joint_x"])[1] # X軸ホイールの回転速度
    #    vel_y = p.getJointState(boxId, joint_map["joint_y"])[1] # Y軸ホイールの回転速度
    #    vel_z = p.getJointState(boxId, joint_map["joint_z"])[1] # Z軸ホイールの回転速度
        # p.getJointState() の戻り値は (位置, 速度, 反力6成分, モータートルク) のタプルで、インデックス [1] が角速度 [rad/s] です。

        return np.array([err_r, err_p, err_y,
                       ang_vel[0], ang_vel[1], ang_vel[2],
                       wvel_x, wvel_y, wvel_z], dtype=np.float32)

#        return np.array([err_r, err_p, err_y,
#                         ang_vel[0], ang_vel[1], ang_vel[2]], dtype=np.float32)
    

# ② 観測の正規化
# 角度の正規化（wrap 処理）が必要な理由
# これをしないと「359° と 1° は 358° 離れている」と誤解されて学習が発散します。 
# _wrap() を静的メソッドとして分離しているのは、for err in [...]: err -= ... のような ループ内再代入（Python では元の変数が変わらないバグパターン）を避けるためです。
#"""    
#    @staticmethod   # 静的メソッドを定義するためのデコレータです。self を引数に取らない関数をクラス内に定義できます。通常の関数のように動作します。
#    def _wrap(a):
#        while a >  math.pi: a -= 2 * math.pi
#        while a < -math.pi: a += 2 * math.pi
#        return a
#"""
    # while ループより NumPy の数式の方が高速で明確：
    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def close(self):
        p.disconnect(physicsClientId=self.physicsClient)
    
    def render(self):
        pass  # PyBullet GUI は __init__ の render_modeで制御済み

#  改善点 ③: _get_obs() の外部呼び出し (g076a line 241)
#  _get_obs()
#  はプライベット規約（アンダースコア）ですが g076a から直接呼ばれています。
# g071 にパブリックメソッドを追加することで意図が明確になります:
    def get_obs(self):
        return self._get_obs()