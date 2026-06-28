import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import xml.etree.ElementTree as ET

#scipy とは？
# scipy は科学技術計算のための Python ライブラリです。 
# 線形代数・最適化・信号処理・統計など、理工学で頻繁に使う計算がひと通り揃っています。 
# 今回使うのはその中の scipy.linalg（線形代数モジュール） です。
try:
    from scipy.linalg import solve_discrete_are
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("!!! scipy not found — using fallback gains !!!")

# ① URDFから物理パラメータを自動取得
# まず、ロボットの物理パラメータを my_cubli.urdf から読み取ります。 
# パラメータをコードに直書きすると、CADを修正するたびにコードも直す必要があります。 
# URDF（設計データ）から自動で読み取ることで、設計変更に強い実装になります
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
# 起動時に以下が表示されます：
# === URDF 取得パラメータ ===
#  全体質量  M  = 0.7000 kg
#  重心距離  L  = 0.0866 m
#  機体慣性  Ib = 0.000600 kg·m²
#  ホイール  Iw = 0.000100 kg·m²
_params = load_urdf_params("day6/my_cubli.urdf")
print("=== URDF 取得パラメータ ===")
print(f"  全体質量  M  = {_params['M']:.4f} kg")
print(f"  重心距離  L  = {_params['L']:.4f} m")
print(f"  機体慣性  Ib = {_params['Ib']:.6f} kg·m²")
print(f"  ホイール  Iw = {_params['Iw']:.6f} kg·m²")
print("=========================")

# 6.1 コントローラクラスの解説
# 制御の中核となる LQRController クラスです。 
# 「物理パラメータを読む 
#  → 状態方程式を立てる 
#   → ブライソン則で重みを決める 
#    → DARE でゲインを算出する」 という流れで、起動時に一度だけ実行される初期化処理です。
# ===== 1. LQRコントローラ =====
class LQIController:
    """
    状態: x = [angle_err, euler_rate, wheel_vel]
    モデル:
      A = [[0,       1, 0],
           [MGL/Ib,  0, 0],
           [-MGL/Ib, 0, 0]]
      B = [[0], [-1/Ib], [(Ib+Iw)/(Ib*Iw)]]
    制御則: u = -K @ x
********ドキュメント・コメントの誤り（動作には影響しないが不正確）
  65〜71行: docstringが3次元のまま
    Q/R はブライソン則で設定:
      Q[i,i] = 1 / (許容最大値_i)^2
      R[0,0] = 1 / (最大トルク)^2
    """
    def __init__(self, params, dt,
                 max_angle_deg=10.0,   # 許容最大角度誤差 [deg]
                 max_rate=2.0,         # 許容最大角速度 [rad/s]
                 max_wheel_vel=100.0,  # 許容最大ホイール角速度 [rad/s]
                 max_torque=0.5,       # 通常動作目標トルク [Nm]（アクチュエータ上限≠）
                 max_integral=0.021):    # ← 追加: 許容最大積分値 [rad·s]
        # ② システム行列 A, B の構築
        # 取得したパラメータを使って、Section 3 で導出した状態方程式 
        # x` = Acx + Bcuを組み立てます。
        self.M  = params["M"]
        self.G  = params["G"]
        self.L  = params["L"]
        self.Ib = params["Ib"]
        self.Iw = params["Iw"]

        MGL = self.M * self.G * self.L # 重力トルク係数 ≈ 0.595
        # 連続時間システム行列
        Ac = np.array([
            [0,            1, 0, 0],   # θ̇ = θ̇      # θ̇ = θ̇（定義）
            [MGL/self.Ib,  0, 0, 0],   # θ̈ = 重力項     # θ̈ = (MGL/Ib)θ ← 重力が倒す方向に加速
            [-MGL/self.Ib, 0, 0, 0],   # φ̈ = ホイール反作用     # φ̈ = -(MGL/Ib)θ ← ホイールが反対に回る
            [1,            0, 0, 0],   # ← 追加: d(∫err)/dt = err
        ])
        Bc = np.array([
            [0],
            [-1 / self.Ib],                                 # ^トルクが本体を押す  
            [(self.Ib + self.Iw) / (self.Ib * self.Iw)],    # ^反作用でホイールが回る
            [0],                        # ← 追加: 積分器はトルクで動かない
        ])
        # 行列の各要素は「今の状態が次の瞬間にどう変化するか」を表しています。 
        # 例えば Ac[1,0] = MGL/Ib は「角度が大きいほど、速く倒れる」という重力の効果です。

        # ③ ブライソン則による Q, R の自動設計
        # LQRの重み行列 Q,R は「何をどこまで許容するか」の仕様書です。
        # ブライソン則はその仕様を数値に変換するレシピです：
        # 「許容値が小さいほどペナルティが大きく、LQRはその状態量を強く抑えようとする」と読めます。
        # ブライソン則: Q[i] = 1/max_i^2, R = 1/max_torque^2
        q_angle = 1.0 / math.radians(max_angle_deg) ** 2
        q_rate  = 1.0 / max_rate ** 2
        q_wheel = 1.0 / max_wheel_vel ** 2
        q_integral = 1.0 / max_integral ** 2              # ← 追加
        r_val   = 1.0 / max_torque ** 2
        #max_torque はアクチュエータ上限ではないことに注意してください。
        # max_torque=0.5 は「通常動作で使いたいトルクの目安」です。
        # アクチュエータの物理上限（2.0 Nm）ではありません。
        #  max_torque=2.0 にすると R が小さくなりゲインが大きくなりすぎ、常にトルクが飽和した「バンバン制御」になります。 
        #（詳細は Section 7.3）
        # Q = np.diag([q_angle, q_rate, q_wheel])
        Q = np.diag([q_angle, q_rate, q_wheel, q_integral])  # 4×4
        R = np.array([[r_val]])

        #④ 離散時間への変換と DARE によるゲイン算出
        # Python で計算したゲインは、240 Hz（1ステップ = 1/240 秒）の離散シミュレーションで使います。 
        # 連続時間のまま使うと数値的に不安定になるため（詳細は Section 7.2）、先に離散時間系に変換します
        # 離散時間系に変換（オイラー法）
        #   一次近似（Forward Euler）による離散化
        # ※この近似は dt が十分小さい場合にのみ有効。
        #   dt が大きい場合や高ゲイン系では離散極が単位円を超えて不安定化する可能性がある。
        #   実機や厳密設計では ZOH（ゼロ次ホールド）離散化 scipy.signal.cont2discrete を推奨。
        # 8.2 連続時間LQRを離散系にそのまま使ってはいけない
        # solve_continuous_are（CARE）で求めたゲインを 240 Hz の離散シミュレーションに直接使うと、 
        # トルクが毎ステップ正負に振動する「ナイキスト振動」が発生することがあります。
        # 必ず solve_discrete_are（DARE）を使い、先に A, B 行列を離散化してください。
        # Ad = np.eye(3) + Ac * dt
        # 試してみよう③ オイラー法を行列指数関数に変える
        from scipy.linalg import expm
        Ad = expm( Ac * dt )
        Bd = Bc * dt

        print(f"  Q = diag({q_angle:.2f}, {q_rate:.4f}, {q_wheel:.6f},{q_integral:.4f}),  R = {r_val:.4f}")  # ←修正

        if HAS_SCIPY:
            try:
                # リカッチ方程式は手で解くと非常に複雑ですが、scipy に渡すだけで一瞬で答えが返ってきます。
                # DARE: 離散時間LQR → 離散極が単位円内に保証される
                # DARE（離散代数リカッチ方程式）を解く
                P = solve_discrete_are(Ad, Bd, Q, R)
                self.K = (np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad)[0]
                # 起動時に 離散極 |z| = [0.9987, 0.8821, 0.3104] のように全ての極が 1 未満であれば安定です。
            except Exception as e:
                print(f"DARE solver failed: {e} — using fallback")
                self.K = np.array([45.0, 5.0, 0.05, 1.0])   # ← 末尾に積分ゲインを追加])
        else:
            self.K = np.array([45.0, 5.0, 0.05, 1.0])   # ← 末尾に積分ゲインを追加])

        # ⑤ 起動時の安定性確認
        # ゲインを計算したら、閉ループの離散極を確認します。 
        # 離散時間系では「全ての極の絶対値が 1 未満」なら安定です
        # （連続時間の「実部が負」に対応）。
        # print(f"  LQR K = [{self.K[0]:.4f}, {self.K[1]:.4f}, {self.K[2]:.4f}]")
        # 極の表示を4次元に
        print(f"  LQI K = [{self.K[0]:.4f}, {self.K[1]:.4f}, {self.K[2]:.4f}, {self.K[3]:.4f}]")    
        Acl = Ad - Bd @ self.K.reshape(1, -1)
        poles = np.abs(np.linalg.eigvals(Acl))
        #print(f"  離散極 |z| = [{poles[0]:.4f}, {poles[1]:.4f}, {poles[2]:.4f}]  (全て<1なら安定)")
        print(f"  離散極 |z| = {[f'{v:.4f}' for v in poles]} (全て<1なら安定)")
    # ⑥ 制御則（1ステップごとに呼ばれる部分）
    # 上の初期化で求めた K を使って、毎ステップのトルクを計算します。 
    # 状態ベクトル x を作って内積を取るだけのシンプルな計算です。
    ### 変更前
    #def compute(self, angle_err, euler_rate, wheel_vel):
    #    state = np.array([angle_err, euler_rate, wheel_vel])
    #    return float(-self.K @ state)    # u = -Kx
    ### 変更後
    def compute(self, angle_err, euler_rate, wheel_vel, integral_err):  #← 引数追加
        state = np.array([angle_err, euler_rate, wheel_vel, integral_err])# ← 4次元
        return float(-self.K @ state)
        #-K @ state の負号が重要です。
        #  例えば角度が正（前に傾いた）なら、K[0] は正なので \(-K[0] \times \text{正} = \text{負\)のトルク（前に押し戻す）になります。
    # 6.2 なぜLQRはPIDより優れているのか
    #  算出されたゲイン（例：K = [21.36, 1.26, 0.03]）を見てください。 
    # これは、PID制御で言うところの以下に対応します。
    #   LQRゲイン	    対応するPID項	            役割
    # K[0]≈ 21.36     Kp（角度への比例項）	        傾きを戻す
    # K[1]≈ 1.26      Kd（角速度への微分項）        倒れる勢いを止める
    # K[2]≈ 0.03      Ki（ホイール速度への積分項）  ホイールの暴走を防ぐ

    #「第3の項」が決定的な違い
    # PID制御は「角度を直す」ことしか考えていません。
    # PID制御では「角度」しか見ていないため、姿勢を直すためにホイールを無限に加速させてしまい、 最終的にモータの限界を超えて転倒します。
    # 一方LQRは、「姿勢を直したいけど、ホイール速度も上げすぎたくない」 というトレードオフを Q行列で考慮してゲインを決めています。 
    # そのため「倒れそうなときは加速して耐えるが、余裕ができたらゆっくり減速してホイール速度を回収する」 という、人間業では調整不可能な高度な制御を自動で行います。

    # なぜ Day 5 の PID より K 値が小さいのか
    # Day 5 の PID は Kp=0.5 でもトルクが飽和していました。
    # LQR の K[0]=21.4 は一見大きく見えますが、 
    # ブライソン則で max_torque=0.5 Nm を指定しているため、
    # 10° 以下の誤差では実際のトルクは 0.2 Nm 前後に収まります。 
    # 「ゲインが大きい＝飽和する」ではなく、ゲインの大きさと許容誤差はセットで理解する必要があります。

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

# ===== 4. LQI 初期化（ブライソン則） =====
_dt = 1. / 240.
print("=== LQI 設計パラメータ（離散時間 DARE） ===")
### 変更前
#lqr_x = LQRController(_params, _dt, max_angle_deg=10.0, max_rate=2.0,
#                       max_wheel_vel=100.0, max_torque=0.5)
#lqr_y = LQRController(_params, _dt, max_angle_deg=10.0, max_rate=2.0,
#                       max_wheel_vel=100.0, max_torque=0.5)
#lqr_z = LQRController(_params, _dt, max_angle_deg=10.0, max_rate=2.0,
#                       max_wheel_vel=100.0, max_torque=0.5)
### 変更後
lqr_x = LQIController(_params, _dt, max_angle_deg=10.0, max_rate=2.0,
                         max_wheel_vel=100.0, max_torque=0.5, max_integral=0.021)
lqr_y = LQIController(_params, _dt, max_angle_deg=10.0, max_rate=2.0,
                         max_wheel_vel=100.0, max_torque=0.5, max_integral=0.021)
lqr_z = LQIController(_params, _dt, max_angle_deg=10.0, max_rate=2.0,
                         max_wheel_vel=100.0, max_torque=0.5, max_integral=0.021)
print("===========================================")

#7.3 GUIパネル
#  PyBullet の画面左にデバッグパネルが表示されます。
#  シミュレーション実行中もリアルタイムで操作できます。
# ===== 5. GUI スライダー =====
sld_max_t   = p.addUserDebugParameter("Max Torque [Nm]",    0,  2.0, 2.0) #スライダー (0〜2.0, 初期値 2.0)	各軸のトルク上限をリアルタイムに変更
btn_start   = p.addUserDebugParameter(">>> START LQI <<<",  1,    0,   0) #ボタン	制御ループ開始
btn_reset   = p.addUserDebugParameter("Reset Robot",        1,    0,   0) #ボタン	初期姿勢に戻してグラフをクリア
btn_dist_r  = p.addUserDebugParameter("Disturbance: Roll",  1,    0,   0) #ボタン	指定軸に外乱トルクを 0.1 秒間（24 ステップ）印加
btn_dist_p  = p.addUserDebugParameter("Disturbance: Pitch", 1,    0,   0) #ボタン	指定軸に外乱トルクを 0.1 秒間（24 ステップ）印加
btn_dist_y  = p.addUserDebugParameter("Disturbance: Yaw",   1,    0,   0) #ボタン	指定軸に外乱トルクを 0.1 秒間（24 ステップ）印加
sld_dist    = p.addUserDebugParameter("Disturbance Torque", 0,  2.0, 0.5) #スライダー (0〜2.0, 初期値 0.5)	外乱トルクの大きさ [Nm]
# トルク上限スライダーで何が確認できるか
# Max Torque を 0.3 Nm 程度に下げると、LQR が計算したトルクが飽和し始めます。 
# このときグラフのトルク波形が「クリッピング」された形になり、 飽和が続くと制御精度が落ちて誤差が増えるのを観察できます。 
# 逆に 2.0 Nm のまま運用すると、通常は 0.2 Nm 前後の余裕のある制御が続きます。

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
# ↓ 追加
integral_err_r = 0.0
integral_err_p = 0.0
integral_err_y = 0.0
INTEGRAL_LIMIT = 0.5   # ワインドアップ防止 [rad·s]
p.resetDebugVisualizerCamera(0.3, 45, -30, [0, 0, 0.05])


# 7.2 リアルタイムグラフ
# 制御中の様子を視覚的に把握するため、Matplotlib で 角度誤差（上段） と 印加トルク（下段） の 2 軸グラフをリアルタイムで表示します。
# データバッファの仕組み
# ===== 6. リアルタイムグラフ =====
PLOT_LEN   = 480    # 直近 480 ステップ（= 2秒分）を保持
PLOT_EVERY = 10     # 10 ステップに 1 回だけ再描画
# 毎ステップ描画しない理由
# 240 Hz で毎回 Matplotlib を更新すると、描画処理が制御ループより重くなりシミュレーションが遅延します。
# PLOT_EVERY = 10 で10ステップに1回（= 24 Hz）に間引くことで、制御周期に影響を与えずにリアルタイムグラフを実現しています。

buf_t  = deque(maxlen=PLOT_LEN)# 時刻
buf_er = deque(maxlen=PLOT_LEN)# Roll 誤差
buf_ep = deque(maxlen=PLOT_LEN)# Pitch 誤差
buf_ey = deque(maxlen=PLOT_LEN)# Yaw 誤差
buf_tx = deque(maxlen=PLOT_LEN)# Roll トルク
buf_ty = deque(maxlen=PLOT_LEN)# Pitch トルク
buf_tz = deque(maxlen=PLOT_LEN)# Yaw トルク
# deque(maxlen=N) は「最新 N 個だけ保持するリスト」です。 
# maxlen を超えると古いデータが自動で削除されるため、 
# メモリを増やさずに「直近 2 秒」のスクロールグラフが実現できます

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

print("Ready. Press 'START LQI' to begin.")

# ===== 7. メインループ =====
try:
    while True:
        # 外乱ボタンX
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
            print("LQI Started!")

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
            integral_err_r = integral_err_p = integral_err_y = 0.0   # ←追加
            for buf in [buf_t, buf_er, buf_ep, buf_ey, buf_tx, buf_ty, buf_tz]:
                buf.clear()
            print("Reset & Paused.")

        if is_running:
            # 姿勢・速度取得
            # PyBulletから現在の姿勢（クォータニオン → オイラー角）と
            # ホイールの回転速度を取得します。
            _, quat = p.getBasePositionAndOrientation(boxId)
            roll, pitch, yaw = p.getEulerFromQuaternion(quat)   # 現在の姿勢

            _, ang_v_world = p.getBaseVelocity(boxId)           # ワールド座標系の角速度

            vel_x = p.getJointState(boxId, joint_map["joint_x"])[1] # X軸ホイールの回転速度
            vel_y = p.getJointState(boxId, joint_map["joint_y"])[1] # Y軸ホイールの回転速度
            vel_z = p.getJointState(boxId, joint_map["joint_z"])[1] # Z軸ホイールの回転速度
            # joint_map はジョイント名（"joint_x" など）とインデックスの辞書で、起動時に getJointInfo で構築します。 
            # 名前でアクセスすることで、URDF の順番変更にも影響を受けません

            # 角度誤差の計算（正規化）
            def wrap(a):
                while a >  math.pi: a -= 2 * math.pi
                while a < -math.pi: a += 2 * math.pi
                return a
            err_r = wrap(roll  - target_roll)   # 目標との差（Roll）
            err_p = wrap(pitch - target_pitch)  # 目標との差（Pitch）
            err_y = wrap(yaw   - target_yaw)    # 目標との差（Yaw）
            # ↓ 追加: 積分更新 + ワインドアップ防止
            integral_err_r += err_r * dt
            integral_err_p += err_p * dt
            integral_err_y += err_y * dt
            integral_err_r = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT,  integral_err_r))
            integral_err_p = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT,  integral_err_p))
            integral_err_y = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT,  integral_err_y))
            # wrap() は誤差を -180° から +180° に正規化します。 
            # これをしないと、例えば 350° と 10° の差が「340°」と計算されてしまい、 
            # 制御器が間違った方向に大トルクを出すことがあります。

            # 角速度の座標変換：ワールド角速度 → オイラー角速度（LQRモデルが仮定するdφ/dt）
            rate_r, rate_p, rate_y = world_angvel_to_euler_rates(ang_v_world, roll, pitch, yaw)
            # LQRのモデルが要求するのは オイラー角速度（dφ/dt, dθ/dt, dψ/dt）ですが、 
            # PyBullet の getBaseVelocity() が返すのは ワールド座標系の角速度 です。
            #  Cubli は (Roll=45°, Pitch=35°) に傾いているため、単純にそのまま使うと軸間の干渉が発生します。
            #  world_angvel_to_euler_rates() で正しく変換します（詳細は Section 7.1）。
            # 8. 実装上の注意点
            #  8.1 角速度フィードバックには座標変換が必要
            #    PyBullet の getBaseVelocity() が返す角速度はワールド座標系です。 
            #    LQR の状態モデルが必要とするのはオイラー角速度(dθ/dt)なので、 そのまま使うと軸間の干渉（クロスカップリング）が発生します。
            # Cubli は (Roll=45°, Pitch=35°) に傾いた状態で立っているため、 ワールド座標の角速度とオイラー角速度の向きは一致しません。
            #   world_angvel_to_euler_rates() で変換してから LQR に渡してください.
            # Gimbal Lock（ジンバルロック）に注意
            #  world_angvel_to_euler_rates() の内部では Pitch 角 Ψ に対して 1/cos(Ψ) の項が現れます。
            #  Pitch = ±90° のとき cos(Ψ) = 0 になり、ゼロ除算（特異点）が発生します。 
            # これをジンバルロックと呼びます。 
            # 今回の Cubli は平衡点が Pitch≈35° であるためジンバルロックは発生しませんが、 
            # 姿勢が大きく傾いて Pitch が ±90° に近づく場合は変換が数値的に発散することを念頭に置いてください。 
            # 実機や大傾角のシステムでは、クォータニオンを直接状態変数にする方法が安全です。

            if step_count == 0:
                print(f"[Init] Roll={math.degrees(roll):.3f}°  "
                      f"Pitch={math.degrees(pitch):.3f}°  Yaw={math.degrees(yaw):.3f}°")
                print(f"[Init] Err  Roll={math.degrees(err_r):.3f}°  "
                      f"Pitch={math.degrees(err_p):.3f}°  Yaw={math.degrees(err_y):.3f}°")
                print(f"[Init] EulerRate Roll={rate_r:.4f}  "
                      f"Pitch={rate_p:.4f}  Yaw={rate_y:.4f} [rad/s]")

            # LQR制御則とトルクの印加
            max_t = p.readUserDebugParameter(sld_max_t) # GUIスライダーからトルク上限を読む

            # joint_x: URDF rpy="0 0 1.5708" → 実効回転軸 = -X → 符号反転
            # joint_y: rpy="0 0 0"           → 実効回転軸 = +Y → そのまま
            # joint_z: rpy="1.5708 0 0"      → 実効回転軸 = +Z → そのまま
            ### 変更前
            #t_x = lqr_x.compute(err_r, rate_r, vel_x) * -1 # joint_x: rpy="0 0 1.5708" → 実効軸が-X
            #t_y = lqr_y.compute(err_p, rate_p, vel_y) *  1 # joint_y: rpy="0 0 0"      → 実効軸が+Y
            #t_z = lqr_z.compute(err_y, rate_y, vel_z) *  1 # joint_z: rpy="1.5708 0 0" → 実効軸が+Z
            ### 変更後
            t_x = lqr_x.compute(err_r, rate_r, vel_x, integral_err_r) * -1
            t_y = lqr_y.compute(err_p, rate_p, vel_y, integral_err_p) *  1
            t_z = lqr_z.compute(err_y, rate_y, vel_z, integral_err_y) *  1
            # トルク飽和処理（モータの物理上限のシミュレーション）
            t_x = max(-max_t, min(max_t, t_x))
            t_y = max(-max_t, min(max_t, t_y))
            t_z = max(-max_t, min(max_t, t_z))

            p.setJointMotorControl2(boxId, joint_map["joint_x"], p.TORQUE_CONTROL, force=t_x)
            p.setJointMotorControl2(boxId, joint_map["joint_y"], p.TORQUE_CONTROL, force=t_y)
            p.setJointMotorControl2(boxId, joint_map["joint_z"], p.TORQUE_CONTROL, force=t_z)
            # joint_x だけ符号が反転する理由
            # URDF の joint_x は rpy="0 0 1.5708" で 90° 回転しています。 
            # このため、ホイールの「ローカル Y 軸」が実際にはワールドの -X 方向 を向いています。 
            # 符号を合わせないと、Roll 方向の制御が逆に働いて発散します。
            err_r_deg = math.degrees(err_r)
            err_p_deg = math.degrees(err_p)
            err_y_deg = math.degrees(err_y)

            t_elapsed += dt
            buf_t.append(t_elapsed)
            buf_er.append(err_r_deg); buf_ep.append(err_p_deg); buf_ey.append(err_y_deg)
            buf_tx.append(t_x);      buf_ty.append(t_y);       buf_tz.append(t_z)

            # グラフ更新処理
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
                    ax.relim()              # データに合わせて軸範囲を再計算
                    ax.autoscale_view()     # 自動スケール適用
                fig.canvas.draw()           # 軸スケールを含めて完全再描画
                fig.canvas.flush_events()   # GUI イベントをフラッシュして画面を更新
                # draw() + flush_events() の組み合わせ
                #  flush_events() だけでは Y 軸のオートスケールが反映されないことがあります。 
                #  先に fig.canvas.draw() を呼ぶことで、スケール計算も含めて正しく更新されます。

            # 外乱印加
            # 外乱ボタンの動作は、dist_count というカウンタで管理しています。 
            # ボタンを押すと dist_count = 24 にセットされ、毎ステップ dist_count -= 1 しながら applyExternalTorque() でトルクを加えます。
            # 24 ステップ後に自動停止します。
            if dist_count > 0:
                p.applyExternalTorque(boxId, -1, dist_torque, p.WORLD_FRAME)
                dist_count -= 1

            # 転倒判定
            if abs(err_r_deg) > 30 or abs(err_p_deg) > 30:
                for i in joint_map.values():
                    p.setJointMotorControl2(boxId, i, p.TORQUE_CONTROL, force=0)
                is_running = False
                print(f"*** FALLEN *** Roll:{err_r_deg:.1f}° Pitch:{err_p_deg:.1f}° → Motors stopped.")
            elif step_count % PLOT_EVERY == 0: #10 == 0:
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

# 達成できたこと
#  ・トルクが飽和せず、なめらかな比例制御になった（0.2 Nm 前後）
#  ・振動・発散なし、安定して頂点立ちを維持
#  ・2.0 Nm のアクチュエータ上限に対して、通常動作では余裕を持った制御
# 定常偏差について（既知の問題）
#  Roll に約 8° の定常偏差が残っています。
#  これは 積分器のない LQR の特性 です。
#  接地点の摩擦や角運動量の蓄積など、モデル化されていない定常的な外乱が存在する場合、 積分器がなければ LQR はその外乱に釣り合う誤差を残したまま安定します。 
# （PID の I ゲインを 0 にして定常偏差が残るのと同じ理屈です。）
#  解決するには LQI（積分拡張 LQR） として状態に∫ Θ dt を追加する必要がありますが、 今回は安定倒立という主目的が達成されているため、許容範囲として受け入れます。
# 9.2 PID（Day 5）vs LQR（Day 6）の比較
#   観点	                PID制御（Day 5）	   LQR（Day 6）
#   ゲイン設定              Kp,Kd を手動で試行錯誤  ブライソン則で物理仕様から自動算出
#   見ている状態量	        角度のみ	            角度・角速度・ホイール速度（3変数）
#   ホイール暴走への対処    できない（暴走しやすい）  ゲインに組み込まれているため抑制できる
#   トルクの使い方	        常に上限付近まで飽和    　通常域（±0.2 Nm 前後）で滑らかに動作
#   定常偏差	            小（I 項あり）	        あり（積分器なし、約 8°）
