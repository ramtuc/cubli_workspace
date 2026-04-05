import pybullet as p
import pybullet_data
import time
import math
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ===== 1. 正規化機能付きPIDコントローラ =====
class PIDController:
    def __init__(self, kp, ki, kd, target=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        error = self.target - current_value

        # 角度の正規化 (-pi ~ +pi) : 180度付近での暴走を防ぐ
        while error > math.pi: error -= 2 * math.pi
        while error < -math.pi: error += 2 * math.pi

        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral

        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative

        self.prev_error = error
        return p_term + i_term + d_term

    def reset(self):
        self.prev_error = 0
        self.integral = 0

# ===== 2. PyBullet 初期設定 =====
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# ターゲット姿勢設定（頂点立ち）
target_roll  = math.pi / 4.0                    # 45°
target_pitch = math.atan(1.0 / math.sqrt(2.0)) # ≈ 35.26°
target_yaw   = 0.0
target_euler = [target_roll, target_pitch, target_yaw]

startPos = [0, 0, 0.15 / math.sqrt(3)]         # ≈ 0.0866m
startOrientation = p.getQuaternionFromEuler(target_euler)

# ロボット読み込み
boxId = p.loadURDF("day5/my_cubli.urdf", startPos, startOrientation)
p.changeDynamics(boxId, -1, lateralFriction=1.0)

# ジョイント情報の取得
joint_map = {}
for i in range(p.getNumJoints(boxId)):
    info = p.getJointInfo(boxId, i)
    name = info[1].decode("utf-8")
    joint_map[name] = i
    p.setJointMotorControl2(boxId, i, p.VELOCITY_CONTROL, force=0) # ブレーキ解除

# ===== 3. GUIスライダーの設置 =====
sld_kp    = p.addUserDebugParameter("Kp",                   0,  2.0,  0.5)
sld_kd    = p.addUserDebugParameter("Kd",                   0,  0.5,  0.1)
sld_max_t = p.addUserDebugParameter("Max Torque",           0,   2.0,  0.5)
btn_start  = p.addUserDebugParameter(">>> START CONTROL <<<", 1, 0, 0)
btn_reset  = p.addUserDebugParameter("Reset Robot",           1, 0, 0)
btn_dist_r = p.addUserDebugParameter("Disturbance: Roll",     1, 0, 0)
btn_dist_p = p.addUserDebugParameter("Disturbance: Pitch",    1, 0, 0)
btn_dist_y = p.addUserDebugParameter("Disturbance: Yaw",      1, 0, 0)
sld_dist   = p.addUserDebugParameter("Disturbance Torque",    0, 2.0, 0.5)

prev_start_val  = p.readUserDebugParameter(btn_start)
prev_reset_val  = p.readUserDebugParameter(btn_reset)
prev_dist_r_val = p.readUserDebugParameter(btn_dist_r)
prev_dist_p_val = p.readUserDebugParameter(btn_dist_p)
prev_dist_y_val = p.readUserDebugParameter(btn_dist_y)

# PID初期化
pid_x = PIDController(0, 0, 0, target=target_roll)
pid_y = PIDController(0, 0, 0, target=target_pitch)
pid_z = PIDController(0, 0, 0, target=target_yaw)

is_running = False
is_reset   = True   # True=リセット後固定, False=転倒後そのまま
dist_torque = [0.0, 0.0, 0.0]  # [roll, pitch, yaw] 外乱トルク (1ステップ分)
DIST_STEPS  = 24               # 外乱を与えるステップ数 (=0.1秒)
dist_count  = 0
dt = 1./240.
prev_vel_x = prev_vel_y = prev_vel_z = 0.0
p.resetDebugVisualizerCamera(0.3, 45, -30, [0, 0, 0.05])

# ===== 4. リアルタイムグラフ設定 =====
PLOT_LEN   = 480          # 表示するステップ数（2秒分）
PLOT_EVERY = 10           # 何ステップごとに再描画するか

buf_t    = deque(maxlen=PLOT_LEN)
buf_er   = deque(maxlen=PLOT_LEN)  # error roll
buf_ep   = deque(maxlen=PLOT_LEN)  # error pitch
buf_ey   = deque(maxlen=PLOT_LEN)  # error yaw
buf_tx   = deque(maxlen=PLOT_LEN)  # torque roll
buf_ty   = deque(maxlen=PLOT_LEN)  # torque pitch
buf_tz   = deque(maxlen=PLOT_LEN)  # torque yaw

plt.ion()
fig = plt.figure(figsize=(10, 6))
gs  = gridspec.GridSpec(2, 1, hspace=0.4)

ax_err = fig.add_subplot(gs[0])
ax_err.set_title("Angle Error [deg]")
ax_err.set_ylabel("deg")
ax_err.axhline(0, color="k", linewidth=0.5)
line_er, = ax_err.plot([], [], label="Roll",  color="tab:red")
line_ep, = ax_err.plot([], [], label="Pitch", color="tab:green")
line_ey, = ax_err.plot([], [], label="Yaw",   color="tab:blue")
ax_err.legend(loc="upper right", fontsize=8)

ax_trq = fig.add_subplot(gs[1])
ax_trq.set_title("Applied Torque [Nm]")
ax_trq.set_ylabel("Nm")
ax_trq.set_xlabel("time [s]")
ax_trq.axhline(0, color="k", linewidth=0.5)
line_tx, = ax_trq.plot([], [], label="Roll",  color="tab:red")
line_ty, = ax_trq.plot([], [], label="Pitch", color="tab:green")
line_tz, = ax_trq.plot([], [], label="Yaw",   color="tab:blue")
ax_trq.legend(loc="upper right", fontsize=8)

plt.show(block=False)
plt.pause(0.01)

step_count = 0
t_elapsed  = 0.0

print("Ready. Press 'START CONTROL' to begin.")

# ===== 5. メインループ =====
try:
    while True:
        # 外乱ボタン
        curr_dist_r = p.readUserDebugParameter(btn_dist_r)
        curr_dist_p = p.readUserDebugParameter(btn_dist_p)
        curr_dist_y = p.readUserDebugParameter(btn_dist_y)
        dist_mag = p.readUserDebugParameter(sld_dist)
        if curr_dist_r != prev_dist_r_val:
            prev_dist_r_val = curr_dist_r
            dist_torque = [dist_mag, 0.0, 0.0]; dist_count = DIST_STEPS
            print(f"Disturbance: Roll  {dist_mag:.2f} Nm")
        if curr_dist_p != prev_dist_p_val:
            prev_dist_p_val = curr_dist_p
            dist_torque = [0.0, dist_mag, 0.0]; dist_count = DIST_STEPS
            print(f"Disturbance: Pitch {dist_mag:.2f} Nm")
        if curr_dist_y != prev_dist_y_val:
            prev_dist_y_val = curr_dist_y
            dist_torque = [0.0, 0.0, dist_mag]; dist_count = DIST_STEPS
            print(f"Disturbance: Yaw   {dist_mag:.2f} Nm")

        # STARTボタン
        curr_start_val = p.readUserDebugParameter(btn_start)
        if curr_start_val != prev_start_val:
            prev_start_val = curr_start_val
            is_running = True
            is_reset   = False
            print("Control Started!")

        # リセットボタン
        curr_reset_val = p.readUserDebugParameter(btn_reset)
        if curr_reset_val != prev_reset_val:
            prev_reset_val = curr_reset_val
            is_running = False
            is_reset   = True
            p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
            p.resetBaseVelocity(boxId, [0,0,0], [0,0,0])
            for i in joint_map.values():
                p.resetJointState(boxId, i, 0, 0)
            pid_x.reset(); pid_y.reset(); pid_z.reset()
            t_elapsed = 0.0
            for buf in [buf_t, buf_er, buf_ep, buf_ey, buf_tx, buf_ty, buf_tz]:
                buf.clear()
            print("Reset & Paused.")

        if is_running:
            # 姿勢取得
            _, quat = p.getBasePositionAndOrientation(boxId)
            r, p_angle, y = p.getEulerFromQuaternion(quat)

            # ゲイン・方向読み取り
            kp    = p.readUserDebugParameter(sld_kp)
            kd    = p.readUserDebugParameter(sld_kd)
            max_t = p.readUserDebugParameter(sld_max_t)
            dir_x =  1
            dir_y = -1
            dir_z = -1

            vel_x = p.getJointState(boxId, joint_map["joint_x"])[1]
            vel_y = p.getJointState(boxId, joint_map["joint_y"])[1]
            vel_z = p.getJointState(boxId, joint_map["joint_z"])[1]

            for pid in [pid_x, pid_y, pid_z]:
                pid.kp, pid.kd = kp, kd

            # PID計算 & トルク適用（各軸ごとに方向を設定）
            t_x = pid_x.compute(r,       dt) * dir_x
            t_y = pid_y.compute(p_angle, dt) * dir_y
            t_z = pid_z.compute(y,       dt) * dir_z

            t_x = max(-max_t, min(max_t, t_x))
            t_y = max(-max_t, min(max_t, t_y))
            t_z = max(-max_t, min(max_t, t_z))

            if "joint_x" in joint_map:
                p.setJointMotorControl2(boxId, joint_map["joint_x"], p.TORQUE_CONTROL, force=t_x)
            if "joint_y" in joint_map:
                p.setJointMotorControl2(boxId, joint_map["joint_y"], p.TORQUE_CONTROL, force=t_y)
            if "joint_z" in joint_map:
                p.setJointMotorControl2(boxId, joint_map["joint_z"], p.TORQUE_CONTROL, force=t_z)

            acc_x = (vel_x - prev_vel_x) / dt
            acc_y = (vel_y - prev_vel_y) / dt
            acc_z = (vel_z - prev_vel_z) / dt
            prev_vel_x, prev_vel_y, prev_vel_z = vel_x, vel_y, vel_z

            err_r = (target_roll  - r)       * 180 / math.pi
            err_p = (target_pitch - p_angle) * 180 / math.pi
            err_y = (target_yaw   - y)       * 180 / math.pi

            # バッファに追記
            t_elapsed += dt
            buf_t.append(t_elapsed)
            buf_er.append(err_r); buf_ep.append(err_p); buf_ey.append(err_y)
            buf_tx.append(t_x);   buf_ty.append(t_y);   buf_tz.append(t_z)

            # グラフ更新（PLOT_EVERYステップごと）
            step_count += 1
            if step_count % PLOT_EVERY == 0:
                t_list = list(buf_t)
                line_er.set_data(t_list, list(buf_er))
                line_ep.set_data(t_list, list(buf_ep))
                line_ey.set_data(t_list, list(buf_ey))
                line_tx.set_data(t_list, list(buf_tx))
                line_ty.set_data(t_list, list(buf_ty))
                line_tz.set_data(t_list, list(buf_tz))
                for ax in [ax_err, ax_trq]:
                    ax.relim()
                    ax.autoscale_view()
                fig.canvas.flush_events()

            # 外乱印加
            if dist_count > 0:
                p.applyExternalTorque(boxId, -1, dist_torque, p.WORLD_FRAME)
                dist_count -= 1

            # 転倒判定: Roll or Pitch の誤差が30°超えたらモータ停止
            if abs(err_r) > 30 or abs(err_p) > 30:
                for i in joint_map.values():
                    p.setJointMotorControl2(boxId, i, p.TORQUE_CONTROL, force=0)
                is_running = False
                print(f"*** FALLEN *** Err Roll:{err_r:.1f} Pitch:{err_p:.1f} → Motors stopped.")
            else:
                print(f"Err Roll:{err_r:5.1f} Pitch:{err_p:5.1f} Yaw:{err_y:5.1f} | "
                      f"acc  Roll:{acc_x:7.1f} Pitch:{acc_y:7.1f} Yaw:{acc_z:7.1f} | "
                      f"Torque Roll:{t_x:6.3f} Pitch:{t_y:6.3f} Yaw:{t_z:6.3f}")

            p.stepSimulation()
        else:
            if is_reset:
                # リセット後: 初期姿勢に固定
                p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
                p.resetBaseVelocity(boxId, [0,0,0], [0,0,0])
                for i in joint_map.values():
                    p.resetJointState(boxId, i, 0, 0)
            else:
                # 転倒後: そのまま物理継続
                p.stepSimulation()

        time.sleep(dt)

except KeyboardInterrupt:
    p.disconnect()
    plt.close()
