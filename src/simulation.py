import pybullet as p
import pybullet_data
import time
import numpy as np

class CubliSimulation:
    def __init__(self, use_gui=True):
        self.use_gui = use_gui
        # 1. PyBullet接続
        if self.use_gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)  # 学習用に画面なしモードも用意しておく

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        
        # 2. 床とCubliの読み込み
        self.planeId = p.loadURDF("plane.urdf")
        self.reset()

    def reset(self):
        """シミュレーションを初期状態に戻す"""
        # スタート位置: 高さ0.08m (地面すれすれ)、角度なし
        startPos = [0, 0, 0.08]
        startOrn = p.getQuaternionFromEuler([0, 0, 0])
        
        # Cubliをロード（既にいる場合はリセット）
        # ※本来は resetBasePositionAndOrientation を使うのが軽いですが、
        #   今回は分かりやすく毎回ロードし直す形にします（Day 3で最適化します）
        if hasattr(self, 'cubliId'):
            p.removeBody(self.cubliId)
            
        self.cubliId = p.loadURDF("urdf/cubli_1axis.urdf", startPos, startOrn)

        # 1. 床の摩擦（滑らないようにグリップさせる）
        p.changeDynamics(self.planeId, -1, lateralFriction=1.0)

        # 2. Cubli本体の摩擦と抵抗
        p.changeDynamics(self.cubliId, -1, 
                         lateralFriction=1.0,  # 床との摩擦係数（滑り止め）
                         linearDamping=0.0,    # 空気抵抗（移動）→ ゼロにする
                         angularDamping=0.0)   # 空気抵抗（回転）→ ゼロにする

        # 3. モーターの内部抵抗（これもゼロにして純粋なトルク制御にする）
        p.changeDynamics(self.cubliId, 0, jointDamping=0.0)
        
        # モーター制御モード設定
        p.setJointMotorControl2(self.cubliId, 0, p.VELOCITY_CONTROL, force=0)
    def apply_torque(self, torque):
        """
        モーターにトルクを加える
        torque: float (Nm)
        """
        # Joint 0 (ボディとホイールの間の関節) にトルク指令を送る
        p.setJointMotorControl2(bodyUniqueId=self.cubliId, 
                                jointIndex=0, 
                                controlMode=p.TORQUE_CONTROL, 
                                force=torque)

    def step(self):
        """時間を1ステップ進める"""
        p.stepSimulation()
        if self.use_gui:
            time.sleep(1./60.)

    def get_state(self):
        """現在の状態（ボディ角度、ホイール速度など）を取得する（デバッグ用）"""
        # ボディの姿勢
        pos, orn = p.getBasePositionAndOrientation(self.cubliId)
        euler = p.getEulerFromQuaternion(orn)
        
        # ホイールの関節状態
        joint_state = p.getJointState(self.cubliId, 0)
        wheel_velocity = joint_state[1]
        
        return euler[0], wheel_velocity  # ボディのRoll角, ホイール角速度