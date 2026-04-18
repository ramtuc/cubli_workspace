import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import xml.etree.ElementTree as ET

try:
    from scipy.linalg import solve_discrete_are
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("!!! scipy not found — using fallback gains !!!")

# ===== URDF から物理パラメータを取得 =====
def load_urdf_params(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    links = {link.get("name"): link for link in root.findall("link")}

    def inertial(link_name):
        el = links[link_name].find("inertial")
        mass = float(el.find("mass").get("value"))
        ie = el.find("inertia")
        return mass, float(ie.get("ixx")), float(ie.get("iyy"))

    m_body, Ib, _ = inertial("base_link")
    m_wheel, Iw, _ = inertial("wheel_y")
    L = 0.15 / math.sqrt(3)
    M = m_body + 3 * m_wheel
    return dict(M=M, G=9.81, L=L, Ib=Ib, Iw=Iw)

_params = load_urdf_params("day6/my_cubli.urdf")
print("=== URDF 取得パラメータ ===")
print(f"  全体質量  M  = {_params['M']:.4f} kg")
print(f"  重心距離  L  = {_params['L']:.4f} m")
print(f"  機体慣性  Ib = {_params['Ib']:.6f} kg·m²")
print(f"  ホイール  Iw = {_params['Iw']:.6f} kg·m²")
print("=========================")

# ===== 1. LQRコントローラ =====
class LQRController:
    """
    状態: x = [angle_err, euler_rate, wheel_vel]
    モデル:
      A = [[0,       1, 0],
           [MGL/Ib,  0, 0],
           [-MGL/Ib, 0, 0]]
      B = [[0], [-1/Ib], [(Ib+Iw)/(Ib*Iw)]]
    制御則: u = -K @ x

    Q/R はブライソン則で設定:
      Q[i,i] = 1 / (許容最大値_i)^2
      R[0,0] = 1 / (最大トルク)^2
    """
    def __init__(self, params, dt,
                 max_angle_deg=10.0,   # 許容最大角度誤差 [deg]
                 max_rate=2.0,         # 許容最大角速度 [rad/s]
                 max_wheel_vel=100.0,  # 許容最大ホイール角速度 [rad/s]
                 max_torque=0.5):      # 通常動作目標トルク [Nm]（アクチュエータ上限≠）
        self.M  = params["M"]
        self.G  = params["G"]
        self.L  = params["L"]
        self.Ib = params["Ib"]
        self.Iw = params["Iw"]

        MGL = self.M * self.G * self.L
        Ac = np.array([
            [0,            1, 0],
            [MGL/self.Ib,  0, 0],
            [-MGL/self.Ib, 0, 0],
        ])
        Bc = np.array([
            [0],
            [-1 / self.Ib],
            [(self.Ib + self.Iw) / (self.Ib * self.Iw)],
        ])

        # ブライソン則: Q[i] = 1/max_i^2, R = 1/max_torque^2
        q_angle = 1.0 / math.radians(max_angle_deg) ** 2
        q_rate  = 1.0 / max_rate ** 2
        q_wheel = 1.0 / max_wheel_vel ** 2
        r_val   = 1.0 / max_torque ** 2

        Q = np.diag([q_angle, q_rate, q_wheel])
        R = np.array([[r_val]])

        # 離散時間系に変換（オイラー法）
        Ad = np.eye(3) + Ac * dt
        Bd = Bc * dt

        print(f"  Q = diag({q_angle:.2f}, {q_rate:.4f}, {q_wheel:.6f}),  R = {r_val:.4f}")

        if HAS_SCIPY:
            try:
                # DARE: 離散時間LQR → 離散極が単位円内に保証される
                P = solve_discrete_are(Ad, Bd, Q, R)
                self.K = (np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad)[0]
            except Exception as e:
                print(f"DARE solver failed: {e} — using fallback")
                self.K = np.array([45.0, 5.0, 0.05])
        else:
            self.K = np.array([45.0, 5.0, 0.05])

        print(f"  LQR K = [{self.K[0]:.4f}, {self.K[1]:.4f}, {self.K[2]:.4f}]")
        Acl = Ad - Bd @ self.K.reshape(1, -1)
        poles = np.abs(np.linalg.eigvals(Acl))
        print(f"  離散極 |z| = [{poles[0]:.4f}, {poles[1]:.4f}, {poles[2]:.4f}]  (全て<1なら安定)")

    def compute(self, angle_err, euler_rate, wheel_vel):
        state = np.array([angle_err, euler_rate, wheel_vel])
        return float(-self.K @ state)

# ===== 2. ワールド角速度 → オイラー角速度 変換 =====
def world_angvel_to_euler_rates(ang_v_world, roll, pitch, yaw):
    """
    PyBullet の getBaseVelocity が返すワールド座標系角速度を
    ZYX オイラー角速度 (roll_rate, pitch_rate, yaw_rate) に変換する。

    ZYX規約: R = Rz(yaw) * Ry(pitch) * Rx(roll)

    Step1: ワールド→機体フレーム  ω_body = R^T @ ω_world
    Step2: 機体角速度→オイラー角速度（逆キネマティクス）
      φ̇ = ωx_b + sin(φ)tan(θ)·ωy_b + cos(φ)tan(θ)·ωz_b
      θ̇ = cos(φ)·ωy_b - sin(φ)·ωz_b
      ψ̇ = sin(φ)/cos(θ)·ωy_b + cos(φ)/cos(θ)·ωz_b
    """
    sr, cr = math.sin(roll),  math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw),   math.cos(yaw)

    wx, wy, wz = ang_v_world

    # R^T (ワールド→機体)
    wx_b = cp*cy*wx + cp*sy*wy - sp*wz
    wy_b = (sr*sp*cy - cr*sy)*wx + (sr*sp*sy + cr*cy)*wy + sr*cp*wz
    wz_b = (cr*sp*cy + sr*sy)*wx + (cr*sp*sy - sr*cy)*wy + cr*cp*wz

    # 機体角速度→オイラー角速度
    tp = sp / cp  # tan(pitch)
    rate_roll  = wx_b + sr*tp*wy_b + cr*tp*wz_b
    rate_pitch = cr*wy_b - sr*wz_b
    rate_yaw   = sr/cp*wy_b + cr/cp*wz_b

    return rate_roll, rate_pitch, rate_yaw

# ===== 3. PyBullet 初期設定 =====
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
p.loadURDF("plane.urdf")

target_roll  = math.pi / 4.0
target_pitch = math.atan(1.0 / math.sqrt(2.0))
target_yaw   = 0.0

startPos         = [0, 0, 0.15 / math.sqrt(3)]
startOrientation = p.getQuaternionFromEuler([target_roll, target_pitch, target_yaw])

boxId = p.loadURDF("day6/my_cubli.urdf", startPos, startOrientation)
p.changeDynamics(boxId, -1, lateralFriction=1.0, spinningFriction=0.1)

joint_map = {}
for i in range(p.getNumJoints(boxId)):
    info = p.getJointInfo(boxId, i)
    name = info[1].decode("utf-8")
    joint_map[name] = i
    p.setJointMotorControl2(boxId, i, p.VELOCITY_CONTROL, force=0)

# ===== 4. LQR 初期化（ブライソン則） =====
_dt = 1. / 240.
print("=== LQR 設計パラメータ（離散時間 DARE） ===")
lqr_x = LQRController(_params, _dt, max_angle_deg=10.0, max_rate=2.0,
                       max_wheel_vel=100.0, max_torque=0.5)
lqr_y = LQRController(_params, _dt, max_angle_deg=10.0, max_rate=2.0,
                       max_wheel_vel=100.0, max_torque=0.5)
lqr_z = LQRController(_params, _dt, max_angle_deg=10.0, max_rate=2.0,
                       max_wheel_vel=100.0, max_torque=0.5)
print("===========================================")

# ===== 5. GUI スライダー =====
sld_max_t   = p.addUserDebugParameter("Max Torque [Nm]",    0,  2.0, 2.0)
btn_start   = p.addUserDebugParameter(">>> START LQR <<<",  1,    0,   0)
btn_reset   = p.addUserDebugParameter("Reset Robot",        1,    0,   0)
btn_dist_r  = p.addUserDebugParameter("Disturbance: Roll",  1,    0,   0)
btn_dist_p  = p.addUserDebugParameter("Disturbance: Pitch", 1,    0,   0)
btn_dist_y  = p.addUserDebugParameter("Disturbance: Yaw",   1,    0,   0)
sld_dist    = p.addUserDebugParameter("Disturbance Torque", 0,  2.0, 0.5)

prev_start_val  = p.readUserDebugParameter(btn_start)
prev_reset_val  = p.readUserDebugParameter(btn_reset)
prev_dist_r_val = p.readUserDebugParameter(btn_dist_r)
prev_dist_p_val = p.readUserDebugParameter(btn_dist_p)
prev_dist_y_val = p.readUserDebugParameter(btn_dist_y)

is_running  = False
is_reset    = True
dist_torque = [0.0, 0.0, 0.0]
DIST_STEPS  = 24
dist_count  = 0
dt = _dt
p.resetDebugVisualizerCamera(0.3, 45, -30, [0, 0, 0.05])


# ===== 6. リアルタイムグラフ =====
PLOT_LEN   = 480
PLOT_EVERY = 10

buf_t  = deque(maxlen=PLOT_LEN)
buf_er = deque(maxlen=PLOT_LEN)
buf_ep = deque(maxlen=PLOT_LEN)
buf_ey = deque(maxlen=PLOT_LEN)
buf_tx = deque(maxlen=PLOT_LEN)
buf_ty = deque(maxlen=PLOT_LEN)
buf_tz = deque(maxlen=PLOT_LEN)

plt.ion()
fig = plt.figure(figsize=(10, 6))
gs  = plt.GridSpec(2, 1, hspace=0.4)

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

print("Ready. Press 'START LQR' to begin.")

# ===== 7. メインループ =====
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

        # START ボタン
        curr_start_val = p.readUserDebugParameter(btn_start)
        if curr_start_val != prev_start_val:
            prev_start_val = curr_start_val
            is_running = True
            is_reset   = False
            step_count = 0
            print("LQR Started!")

        # リセットボタン
        curr_reset_val = p.readUserDebugParameter(btn_reset)
        if curr_reset_val != prev_reset_val:
            prev_reset_val = curr_reset_val
            is_running = False
            is_reset   = True
            p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
            p.resetBaseVelocity(boxId, [0, 0, 0], [0, 0, 0])
            for i in joint_map.values():
                p.resetJointState(boxId, i, 0, 0)
            t_elapsed = 0.0
            for buf in [buf_t, buf_er, buf_ep, buf_ey, buf_tx, buf_ty, buf_tz]:
                buf.clear()
            print("Reset & Paused.")

        if is_running:
            # 姿勢・速度取得
            _, quat = p.getBasePositionAndOrientation(boxId)
            roll, pitch, yaw = p.getEulerFromQuaternion(quat)

            _, ang_v_world = p.getBaseVelocity(boxId)

            vel_x = p.getJointState(boxId, joint_map["joint_x"])[1]
            vel_y = p.getJointState(boxId, joint_map["joint_y"])[1]
            vel_z = p.getJointState(boxId, joint_map["joint_z"])[1]

            # 角度誤差
            def wrap(a):
                while a >  math.pi: a -= 2 * math.pi
                while a < -math.pi: a += 2 * math.pi
                return a
            err_r = wrap(roll  - target_roll)
            err_p = wrap(pitch - target_pitch)
            err_y = wrap(yaw   - target_yaw)

            # ワールド角速度 → オイラー角速度（LQRモデルが仮定するdφ/dt）
            rate_r, rate_p, rate_y = world_angvel_to_euler_rates(ang_v_world, roll, pitch, yaw)

            if step_count == 0:
                print(f"[Init] Roll={math.degrees(roll):.3f}°  "
                      f"Pitch={math.degrees(pitch):.3f}°  Yaw={math.degrees(yaw):.3f}°")
                print(f"[Init] Err  Roll={math.degrees(err_r):.3f}°  "
                      f"Pitch={math.degrees(err_p):.3f}°  Yaw={math.degrees(err_y):.3f}°")
                print(f"[Init] EulerRate Roll={rate_r:.4f}  "
                      f"Pitch={rate_p:.4f}  Yaw={rate_y:.4f} [rad/s]")

            # LQR 制御則
            max_t = p.readUserDebugParameter(sld_max_t)

            # joint_x: URDF rpy="0 0 1.5708" → 実効回転軸 = -X → 符号反転
            # joint_y: rpy="0 0 0"           → 実効回転軸 = +Y → そのまま
            # joint_z: rpy="1.5708 0 0"      → 実効回転軸 = +Z → そのまま
            t_x = lqr_x.compute(err_r, rate_r, vel_x) * -1
            t_y = lqr_y.compute(err_p, rate_p, vel_y) *  1
            t_z = lqr_z.compute(err_y, rate_y, vel_z) *  1

            t_x = max(-max_t, min(max_t, t_x))
            t_y = max(-max_t, min(max_t, t_y))
            t_z = max(-max_t, min(max_t, t_z))

            p.setJointMotorControl2(boxId, joint_map["joint_x"], p.TORQUE_CONTROL, force=t_x)
            p.setJointMotorControl2(boxId, joint_map["joint_y"], p.TORQUE_CONTROL, force=t_y)
            p.setJointMotorControl2(boxId, joint_map["joint_z"], p.TORQUE_CONTROL, force=t_z)

            err_r_deg = math.degrees(err_r)
            err_p_deg = math.degrees(err_p)
            err_y_deg = math.degrees(err_y)

            t_elapsed += dt
            buf_t.append(t_elapsed)
            buf_er.append(err_r_deg); buf_ep.append(err_p_deg); buf_ey.append(err_y_deg)
            buf_tx.append(t_x);      buf_ty.append(t_y);       buf_tz.append(t_z)

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
                fig.canvas.draw()
                fig.canvas.flush_events()

            # 外乱印加
            if dist_count > 0:
                p.applyExternalTorque(boxId, -1, dist_torque, p.WORLD_FRAME)
                dist_count -= 1

            # 転倒判定
            if abs(err_r_deg) > 30 or abs(err_p_deg) > 30:
                for i in joint_map.values():
                    p.setJointMotorControl2(boxId, i, p.TORQUE_CONTROL, force=0)
                is_running = False
                print(f"*** FALLEN *** Roll:{err_r_deg:.1f}° Pitch:{err_p_deg:.1f}° → Motors stopped.")
            elif step_count % 1 == 10:
                print(f"Err Roll:{err_r_deg:5.1f} Pitch:{err_p_deg:5.1f} Yaw:{err_y_deg:5.1f} | "
                      f"Torque Roll:{t_x:6.3f} Pitch:{t_y:6.3f} Yaw:{t_z:6.3f}")

            p.stepSimulation()
        else:
            if is_reset:
                p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
                p.resetBaseVelocity(boxId, [0, 0, 0], [0, 0, 0])
                for i in joint_map.values():
                    p.resetJointState(boxId, i, 0, 0)
            else:
                p.stepSimulation()

        time.sleep(dt)

except KeyboardInterrupt:
    p.disconnect()
    plt.close()
