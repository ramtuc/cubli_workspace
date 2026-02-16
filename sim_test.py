# sim_test.py
import pybullet as p
import pybullet_data
import time
import math

# GUIモードで物理エンジンを起動
try:
    physicsClient = p.connect(p.GUI)
except Exception as e:
    print("Failed to connect to PyBullet GUI:", e)
    exit()

# 基本データの読み込み設定
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 重力の設定（地球と同じ -9.81 m/s^2）
p.setGravity(0, 0, -9.81)

# 床の読み込み
planeId = p.loadURDF("plane.urdf")

# カメラ位置の設定
p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0,0,0])

# 箱を出現させる
cubeStartPos = [0, 0, 0.5] # 高さ50cmから
angle_45deg = math.pi / 4
# あえて45度傾けて、不安定な状態で出現させる
cubeStartOrientation = p.getQuaternionFromEuler([angle_45deg, angle_45deg, 0])
boxId = p.loadURDF("cube.urdf", cubeStartPos, cubeStartOrientation, globalScaling=0.1)

# 箱の色を赤に変更
p.changeVisualShape(boxId, -1, rgbaColor=[1, 0, 0, 1]) 

print("------------------------------------------------")
print(" 頂点から落下します！")
print(" [SPACE] キーでジャンプします")
print("------------------------------------------------")

# シミュレーションループ
while True:
    p.stepSimulation()

    keys = p.getKeyboardEvents()
    # スペースキーでジャンプ
    if 32 in keys and keys[32] & p.KEY_WAS_TRIGGERED:
        print("Jump!")
        # 真上に強い力を加える (z軸方向に110N)
        p.applyExternalForce(boxId, -1, forceObj=[0, 0, 110], posObj=[0, 0, 0], flags=p.LINK_FRAME)
        # ランダムな回転トルクも加える
        p.applyExternalTorque(boxId, -1, torqueObj=[5, 5, 0.1], flags=p.LINK_FRAME)

    time.sleep(1./240.) # 物理演算速度の調整

# 接続の切断
p.disconnect()