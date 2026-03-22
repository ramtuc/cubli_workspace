import pybullet as p
import pybullet_data
import time
import math

# ===== PyBullet GUI起動 =====
try:
    physicsClient = p.connect(p.GUI)
except Exception as e:
    print("Failed to connect to PyBullet GUI:", e)
    exit()

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 重力設定（地球標準重力 -9.81 m/s²）
p.setGravity(0, 0, -9.81)

# 床を読み込む
planeId = p.loadURDF("plane.urdf")

# カメラ設定（接写で詳細が見えるように）
p.resetDebugVisualizerCamera(
    cameraDistance=0.2,       # カメラ距離 [m]
    cameraYaw=0,              # 水平回転角
    cameraPitch=-30,          # 仰俯角
    cameraTargetPosition=[0, 0, 0.05]  # 注目点
)

boxId = None
startPos = [0, 0, 0.1]       # 初期位置（地面から10cm上）
angle_45deg = math.pi / 4

# 少し傾けた状態で出現させる（姿勢制御のテストのため）
startOrientation = p.getQuaternionFromEuler(
    [angle_45deg * 1, angle_45deg, angle_45deg]
)

# ===== フラグ管理 =====
is_paused = False    # 一時停止フラグ
is_motor_on = False  # モーター回転フラグ

print("------------------------------------------------")
print(" [a] 出現 / リセット")
print(" [p] 一時停止 / 再開")
print(" [m] モーター回転 ON/OFF")
print(" [SPACE] ジャンプ（外力を加える）")
print("------------------------------------------------")

while True:
    keys = p.getKeyboardEvents()

    # ===== 'p' キーで一時停止 / 再開 =====
    if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_TRIGGERED:
        is_paused = not is_paused
        print("Paused!" if is_paused else "Resumed!")

    # ===== 'm' キーでモーター ON/OFF =====
    if ord('m') in keys and keys[ord('m')] & p.KEY_WAS_TRIGGERED:
        is_motor_on = not is_motor_on
        state_str = "ON (Spinning!)" if is_motor_on else "OFF (Coasting)"
        print(f"Motor Torque: {state_str}")

    # ===== 'a' キーで出現 / リセット =====
    if ord('a') in keys and keys[ord('a')] & p.KEY_WAS_TRIGGERED:
        if boxId is not None:
            p.removeBody(boxId)

        # URDFを読み込んでロボットを召喚
        boxId = p.loadURDF("day4/my_cubli.urdf", startPos, startOrientation)

        # ブレーキ解除（VELOCITY_CONTROL の force=0 でフリーにする）
        # これをしないと TORQUE_CONTROL が効かない
        for joint_idx in range(3):
            p.setJointMotorControl2(
                boxId, joint_idx, p.VELOCITY_CONTROL, force=0
            )

        is_motor_on = False
        print("Spawned!")

    # ===== スペースキーでジャンプ（外乱テスト）=====
    if 32 in keys and keys[32] & p.KEY_WAS_TRIGGERED:
        if boxId is not None:
            print("Jump!")
            # 外力を加えてランダムな姿勢にする
            p.applyExternalForce(
                boxId, -1,
                forceObj=[150, 150, 150],
                posObj=[0, 0, 0],
                flags=p.LINK_FRAME
            ) 
            p.applyExternalTorque(
                boxId, -1,
                torqueObj=[5, 5, 0.1],
                flags=p.LINK_FRAME
            )

    # ===== シミュレーションステップ =====
    if not is_paused:
        if boxId is not None:
            # ON なら 0.15Nm、OFF なら 0Nm
            target_torque = 0.15 if is_motor_on else 0.0

            # 3軸すべてに同じトルクを印加
            p.setJointMotorControl2(
                boxId, 0, p.TORQUE_CONTROL, force=target_torque  # joint_x
            )
            p.setJointMotorControl2(
                boxId, 1, p.TORQUE_CONTROL, force=target_torque  # joint_y
            )
            p.setJointMotorControl2(
                boxId, 2, p.TORQUE_CONTROL, force=target_torque  # joint_z
            )

        p.stepSimulation()

    # 240Hzでシミュレーションを進める
    time.sleep(1. / 240.)

p.disconnect()