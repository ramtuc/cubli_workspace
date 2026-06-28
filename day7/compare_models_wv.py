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

import pybullet as p
import pybullet_data
import numpy as np
import math
import time
import os
from collections import deque
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from cubli_env_wv import CubliEnv
import sys
# Robust と Normal を 1 つの PyBullet GUI に並べ、同一タイミング・同一外乱で挙動を比較するスクリプトです。

MODEL_ROBUST_PATH = "models/PPO_Robust_WV/final_model"
STATS_ROBUST_PATH = "models/PPO_Robust_WV/vec_normalize.pkl"
MODEL_NORMAL_PATH = "models/PPO_Normal_WV/final_model"
STATS_NORMAL_PATH = "models/PPO_Normal_WV/vec_normalize.pkl"

TARGET_ROLL  = math.pi / 4.0
TARGET_PITCH = math.atan(1.0 / math.sqrt(2.0))
TARGET_YAW   = 0.0
HEIGHT       = 0.15 / math.sqrt(3)
MAX_TORQUE   = 0.5
DT           = 1.0 / 240.0

PLOT_LEN   = 480
PLOT_EVERY = 10
DIST_STEPS = 24

# 2台のロボットの配置（X方向にオフセット）
POS_ROBUST = [-0.3, 0.0, HEIGHT]
POS_NORMAL = [ 0.3, 0.0, HEIGHT]


def get_obs(boxId, client):
    _, quat   = p.getBasePositionAndOrientation(boxId, physicsClientId=client)
    roll, pitch, yaw = p.getEulerFromQuaternion(quat, physicsClientId=client)
    _, ang_vel = p.getBaseVelocity(boxId, physicsClientId=client)

    wrap = CubliEnv._wrap
#    def wrap(a):
#        while a >  math.pi: a -= 2 * math.pi
#        while a < -math.pi: a += 2 * math.pi
#        return a

#    return np.array([
#        wrap(roll - TARGET_ROLL), wrap(pitch - TARGET_PITCH), wrap(yaw - TARGET_YAW),
#        ang_vel[0], ang_vel[1], ang_vel[2],
#    ], dtype=np.float32)
    wvel_x = p.getJointState(boxId, 0, physicsClientId=client)[1]
    wvel_y = p.getJointState(boxId, 1, physicsClientId=client)[1]
    wvel_z = p.getJointState(boxId, 2, physicsClientId=client)[1]

    return np.array([
        wrap(roll - TARGET_ROLL), wrap(pitch - TARGET_PITCH), wrap(yaw - TARGET_YAW),
        ang_vel[0], ang_vel[1], ang_vel[2],
        wvel_x, wvel_y, wvel_z,
    ], dtype=np.float32)


def apply_action(boxId, action, client):
    torque = np.array(action).flatten() * MAX_TORQUE * -1.0
    for i, t in enumerate(torque):
        p.setJointMotorControl2(boxId, i, p.TORQUE_CONTROL, force=t, physicsClientId=client)


def reset_robot(boxId, pos, client):
    ori = p.getQuaternionFromEuler([TARGET_ROLL, TARGET_PITCH, TARGET_YAW], physicsClientId=client)
    p.resetBasePositionAndOrientation(boxId, pos, ori, physicsClientId=client)
    p.resetBaseVelocity(boxId, [0, 0, 0], [0, 0, 0], physicsClientId=client)
    for i in range(3):
        p.resetJointState(boxId, i, 0, 0, physicsClientId=client)
        p.setJointMotorControl2(boxId, i, p.VELOCITY_CONTROL, force=0, physicsClientId=client)


def setup_buttons(client):
    btns = {
        'start':    p.addUserDebugParameter(">>> START <<<",             1,   0,   0, physicsClientId=client),
        'reset':    p.addUserDebugParameter("Reset",                     1,   0,   0, physicsClientId=client),
        'dist_r':   p.addUserDebugParameter("Disturbance: Roll",         1,   0,   0, physicsClientId=client),
        'dist_p':   p.addUserDebugParameter("Disturbance: Pitch",        1,   0,   0, physicsClientId=client),
        'dist_y':   p.addUserDebugParameter("Disturbance: Yaw",          1,   0,   0, physicsClientId=client),
        'dist_mag': p.addUserDebugParameter("Disturbance Torque [Nm]",   0, 2.5, 0.5, physicsClientId=client),
    }
    prev = {k: p.readUserDebugParameter(v, physicsClientId=client) for k, v in btns.items()}
    return btns, prev


if __name__ == "__main__":
    for path, label in [
        (MODEL_ROBUST_PATH + ".zip", "train_robust.py"),
        (MODEL_NORMAL_PATH + ".zip", "train_normal.py"),
    ]:
        if not os.path.exists(path):
            print(f"モデルが見つかりません: {path}  先に {label} を実行してください。")
            sys.exit(1)

    # VecNormalize と PPO のロード（DIRECT ダミーenv はスペース定義にのみ使用）
    dummy_r = CubliEnv(render_mode=None)
    vec_r   = VecNormalize.load(STATS_ROBUST_PATH, DummyVecEnv([lambda: dummy_r]))
    vec_r.training = False
    vec_r.norm_reward = False
    model_r = PPO.load(MODEL_ROBUST_PATH, env=vec_r)

    dummy_n = CubliEnv(render_mode=None)
    vec_n   = VecNormalize.load(STATS_NORMAL_PATH, DummyVecEnv([lambda: dummy_n]))
    vec_n.training = False
    vec_n.norm_reward = False
    model_n = PPO.load(MODEL_NORMAL_PATH, env=vec_n)

    # PyBullet GUI（1つのシミュレーションに2台）
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=client)
    p.loadURDF("plane.urdf", physicsClientId=client)

    start_ori = p.getQuaternionFromEuler([TARGET_ROLL, TARGET_PITCH, TARGET_YAW], physicsClientId=client)
    boxId_r = p.loadURDF("day7/my_cubli.urdf", POS_ROBUST, start_ori, physicsClientId=client)
    boxId_n = p.loadURDF("day7/my_cubli.urdf", POS_NORMAL, start_ori, physicsClientId=client)

    for boxId in [boxId_r, boxId_n]:
        p.changeDynamics(boxId, -1, lateralFriction=1.0, spinningFriction=0.1, physicsClientId=client)
        for i in range(3):
            p.setJointMotorControl2(boxId, i, p.VELOCITY_CONTROL, force=0, physicsClientId=client)

    # ラベル表示
    p.addUserDebugText("Robust", [POS_ROBUST[0], POS_ROBUST[1], HEIGHT + 0.15],
                       [1.0, 0.3, 0.3], 1.5, physicsClientId=client)
    p.addUserDebugText("Normal", [POS_NORMAL[0], POS_NORMAL[1], HEIGHT + 0.15],
                       [0.3, 0.5, 1.0], 1.5, physicsClientId=client)

    p.resetDebugVisualizerCamera(0.9, 0, -25, [0, 0, 0.1], physicsClientId=client)

    btns, prev = setup_buttons(client)

    is_running = False
    fallen_r   = fallen_n = False
    dist_torque = np.zeros(3, dtype=np.float32)
    dist_count  = 0
    t_elapsed   = 0.0
    step_count  = 0

    buf_t    = deque(maxlen=PLOT_LEN)
    buf_r_r  = deque(maxlen=PLOT_LEN)
    buf_r_p  = deque(maxlen=PLOT_LEN)
    buf_n_r  = deque(maxlen=PLOT_LEN)
    buf_n_p  = deque(maxlen=PLOT_LEN)
    buf_r_tx = deque(maxlen=PLOT_LEN)
    buf_r_ty = deque(maxlen=PLOT_LEN)
    buf_n_tx = deque(maxlen=PLOT_LEN)
    buf_n_ty = deque(maxlen=PLOT_LEN)

    plt.ion()
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    ax_roll, ax_pitch, ax_tx, ax_ty = axes
    fig.suptitle("Comparison: Robust (red) vs Normal (blue dashed)")
    plt.subplots_adjust(hspace=0.45)

    for ax, title, ylabel in [
        (ax_roll,  "Roll Error [deg]",      "deg"),
        (ax_pitch, "Pitch Error [deg]",     "deg"),
        (ax_tx,    "Torque joint_x [Nm]",   "Nm"),
        (ax_ty,    "Torque joint_y [Nm]",   "Nm"),
    ]:
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.axhline(0, color="k", linewidth=0.5)

    line_rr,  = ax_roll.plot( [], [], label="Robust", color="tab:red",  linewidth=1.5)
    line_nr,  = ax_roll.plot( [], [], label="Normal", color="tab:blue", linewidth=1.5, linestyle="--")
    line_rp,  = ax_pitch.plot([], [], label="Robust", color="tab:red",  linewidth=1.5)
    line_np,  = ax_pitch.plot([], [], label="Normal", color="tab:blue", linewidth=1.5, linestyle="--")
    line_rtx, = ax_tx.plot(   [], [], label="Robust", color="tab:red",  linewidth=1.5)
    line_ntx, = ax_tx.plot(   [], [], label="Normal", color="tab:blue", linewidth=1.5, linestyle="--")
    line_rty, = ax_ty.plot(   [], [], label="Robust", color="tab:red",  linewidth=1.5)
    line_nty, = ax_ty.plot(   [], [], label="Normal", color="tab:blue", linewidth=1.5, linestyle="--")
    for ax in axes:
        ax.legend(loc="upper right", fontsize=8)
    ax_ty.set_xlabel("time [s]")

    plt.show(block=False)
    plt.pause(0.01)

    print("Ready. PyBullet に2台のロボットが表示されています。")
    print("  左: Robust モデル  /  右: Normal モデル")
    print("Press '>>> START <<<' to begin.")
    # ループ前に初期化が必要
    _did_idle_reset = False
    try:
        while True:
            curr_start  = p.readUserDebugParameter(btns['start'],    physicsClientId=client)
            curr_reset  = p.readUserDebugParameter(btns['reset'],    physicsClientId=client)
            curr_dist_r = p.readUserDebugParameter(btns['dist_r'],   physicsClientId=client)
            curr_dist_p = p.readUserDebugParameter(btns['dist_p'],   physicsClientId=client)
            curr_dist_y = p.readUserDebugParameter(btns['dist_y'],   physicsClientId=client)
            dist_mag    = p.readUserDebugParameter(btns['dist_mag'], physicsClientId=client)

            if curr_start != prev['start']:
                prev['start'] = curr_start
                is_running = True
                fallen_r = fallen_n = False
                _did_idle_reset = False
                print("Both models started!")

            if curr_reset != prev['reset']:
                prev['reset'] = curr_reset
                is_running = False
                fallen_r = fallen_n = False
                _did_idle_reset = True
                reset_robot(boxId_r, POS_ROBUST, client)
                reset_robot(boxId_n, POS_NORMAL, client)
                t_elapsed  = 0.0
                step_count = 0
                dist_count = 0
                for buf in [buf_t, buf_r_r, buf_r_p, buf_n_r, buf_n_p,
                            buf_r_tx, buf_r_ty, buf_n_tx, buf_n_ty]:
                    buf.clear()
                print("Reset & Paused.")

            if curr_dist_r != prev['dist_r']:
                prev['dist_r'] = curr_dist_r
                dist_torque = np.array([dist_mag / MAX_TORQUE, 0.0, 0.0], dtype=np.float32)
                dist_count  = DIST_STEPS
                print(f"Disturbance: Roll  {dist_mag:.2f} Nm")

            if curr_dist_p != prev['dist_p']:
                prev['dist_p'] = curr_dist_p
                dist_torque = np.array([0.0, dist_mag / MAX_TORQUE, 0.0], dtype=np.float32)
                dist_count  = DIST_STEPS
                print(f"Disturbance: Pitch {dist_mag:.2f} Nm")

            if curr_dist_y != prev['dist_y']:
                prev['dist_y'] = curr_dist_y
                dist_torque = np.array([0.0, 0.0, dist_mag / MAX_TORQUE], dtype=np.float32)
                dist_count  = DIST_STEPS
                print(f"Disturbance: Yaw   {dist_mag:.2f} Nm")

            if is_running:
                obs_r_raw = get_obs(boxId_r, client)
                obs_n_raw = get_obs(boxId_n, client)
                # 各モデルの VecNormalize を別々にロードし、観測を個別に正規化
                obs_r_norm = vec_r.normalize_obs(obs_r_raw.reshape(1, -1))
                obs_n_norm = vec_n.normalize_obs(obs_n_raw.reshape(1, -1))

                action_r, _ = model_r.predict(obs_r_norm, deterministic=True)
                action_n, _ = model_n.predict(obs_n_norm, deterministic=True)
                # 9.2 同一シードで公平な比較をする
                # Normal vs Robust の比較テストでは、まったく同じタイミング・同じ方向の外乱を与えることが重要です。
                # ボタン操作で両機に同時に外乱を注入 → 公平な比較条件を保証
                if dist_count > 0:
                    action_r[0] += dist_torque      # Robust・Normal 両方に同じ外乱を加算
                    action_n[0] += dist_torque      # Robust・Normal 両方に同じ外乱を加算
                    dist_count -= 1

                torques_r = action_r[0] * MAX_TORQUE * -1.0
                torques_n = action_n[0] * MAX_TORQUE * -1.0

                if not fallen_r:
                    apply_action(boxId_r, action_r, client)
                if not fallen_n:
                    apply_action(boxId_n, action_n, client)

                p.stepSimulation(physicsClientId=client)
                time.sleep(DT)

                t_elapsed += DT
                buf_t.append(t_elapsed)
                buf_r_r.append(math.degrees(float(obs_r_raw[0])))
                buf_r_p.append(math.degrees(float(obs_r_raw[1])))
                buf_n_r.append(math.degrees(float(obs_n_raw[0])))
                buf_n_p.append(math.degrees(float(obs_n_raw[1])))
                buf_r_tx.append(float(torques_r[0]))
                buf_r_ty.append(float(torques_r[1]))
                buf_n_tx.append(float(torques_n[0]))
                buf_n_ty.append(float(torques_n[1]))

                if not fallen_r and (abs(obs_r_raw[0]) > 0.8 or abs(obs_r_raw[1]) > 0.8):
                    fallen_r = True
                    print(f"*** Robust 転倒 *** t={t_elapsed:.2f}s")
                if not fallen_n and (abs(obs_n_raw[0]) > 0.8 or abs(obs_n_raw[1]) > 0.8):
                    fallen_n = True
                    print(f"*** Normal 転倒 *** t={t_elapsed:.2f}s")

                step_count += 1
                if step_count % PLOT_EVERY == 0:
                    t_list = list(buf_t)
                    line_rr.set_data( t_list, list(buf_r_r))
                    line_nr.set_data( t_list, list(buf_n_r))
                    line_rp.set_data( t_list, list(buf_r_p))
                    line_np.set_data( t_list, list(buf_n_p))
                    line_rtx.set_data(t_list, list(buf_r_tx))
                    line_ntx.set_data(t_list, list(buf_n_tx))
                    line_rty.set_data(t_list, list(buf_r_ty))
                    line_nty.set_data(t_list, list(buf_n_ty))
                    for ax in axes:
                        ax.relim()
                        ax.autoscale_view()
                    fig.canvas.draw()
                    fig.canvas.flush_events()
            else:
                if not _did_idle_reset:
                    reset_robot(boxId_r, POS_ROBUST, client)
                    reset_robot(boxId_n, POS_NORMAL, client)
                    _did_idle_reset = True
                p.stepSimulation(physicsClientId=client)
                time.sleep(DT)

    except KeyboardInterrupt:
        pass
    finally:
        dummy_r.close()
        dummy_n.close()
        p.disconnect(physicsClientId=client)
        plt.close()
