import pybullet as p
import math
import time
import os
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from cubli_env import CubliEnv

MODEL_PATH = "models/PPO_Normal/final_model"
STATS_PATH = "models/PPO_Normal/vec_normalize.pkl"
TITLE      = "PPO Normal"

PLOT_LEN   = 480
PLOT_EVERY = 100
DIST_STEPS = 24  # 外乱の継続ステップ数


def setup_buttons(client):
    btns = {
        'start':    p.addUserDebugParameter(">>> START PPO <<<",        1,   0,   0, physicsClientId=client),
        'reset':    p.addUserDebugParameter("Reset Robot",              1,   0,   0, physicsClientId=client),
        'dist_r':   p.addUserDebugParameter("Disturbance: Roll",        1,   0,   0, physicsClientId=client),
        'dist_p':   p.addUserDebugParameter("Disturbance: Pitch",       1,   0,   0, physicsClientId=client),
        'dist_y':   p.addUserDebugParameter("Disturbance: Yaw",         1,   0,   0, physicsClientId=client),
        'dist_mag': p.addUserDebugParameter("Disturbance Torque [Nm]",  0, 2.0, 0.5, physicsClientId=client),
    }
    prev = {k: p.readUserDebugParameter(v, physicsClientId=client) for k, v in btns.items()}
    return btns, prev


if __name__ == "__main__":
    if not os.path.exists(MODEL_PATH + ".zip") or not os.path.exists(STATS_PATH):
        print(f"モデルが見つかりません。先に train_normal.py を実行してください。")
        exit()

    base_env = CubliEnv(render_mode="human")
    client   = base_env.physicsClient

    vec_env = DummyVecEnv([lambda: base_env])
    vec_env = VecNormalize.load(STATS_PATH, vec_env)
    vec_env.training    = False
    vec_env.norm_reward = False

    model = PPO.load(MODEL_PATH, env=vec_env)

    obs = vec_env.reset()
    p.resetDebugVisualizerCamera(0.4, 45, -20, [0, 0, 0.1], physicsClientId=client)

    btns, prev = setup_buttons(client)

    is_running  = False
    dist_torque = np.zeros(3, dtype=np.float32)
    dist_count  = 0
    t_elapsed   = 0.0
    step_count  = 0

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
    fig.suptitle(TITLE)

    ax_err = fig.add_subplot(gs[0])
    ax_err.set_title("Angle Error [deg]")
    ax_err.set_ylabel("deg")
    ax_err.axhline(0, color="k", linewidth=0.5)
    line_er, = ax_err.plot([], [], label="Roll",  color="tab:red")
    line_ep, = ax_err.plot([], [], label="Pitch", color="tab:green")
    line_ey, = ax_err.plot([], [], label="Yaw",   color="tab:blue")
    ax_err.legend(loc="upper right", fontsize=8)

    ax_trq = fig.add_subplot(gs[1])
    ax_trq.set_title("Motor Torque [Nm]")
    ax_trq.set_ylabel("Nm")
    ax_trq.set_xlabel("time [s]")
    ax_trq.axhline(0, color="k", linewidth=0.5)
    line_tx, = ax_trq.plot([], [], label="joint_x (Roll)",  color="tab:red")
    line_ty, = ax_trq.plot([], [], label="joint_y (Pitch)", color="tab:green")
    line_tz, = ax_trq.plot([], [], label="joint_z (Yaw)",   color="tab:blue")
    ax_trq.legend(loc="upper right", fontsize=8)

    plt.show(block=False)
    plt.pause(0.01)

    print(f"Ready ({TITLE}). Press '>>> START PPO <<<' to begin.")

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
                print("PPO Started!")

            if curr_reset != prev['reset']:
                is_running = False
                obs = vec_env.reset()
                p.resetDebugVisualizerCamera(0.4, 45, -20, [0, 0, 0.1], physicsClientId=client)
                btns, prev = setup_buttons(client)
                t_elapsed  = 0.0
                step_count = 0
                dist_count = 0
                for buf in [buf_t, buf_er, buf_ep, buf_ey, buf_tx, buf_ty, buf_tz]:
                    buf.clear()
                print("Reset & Paused.")
                continue

            if curr_dist_r != prev['dist_r']:
                prev['dist_r'] = curr_dist_r
                dist_torque = np.array([dist_mag / base_env.max_torque, 0.0, 0.0], dtype=np.float32)
                dist_count  = DIST_STEPS
                print(f"Disturbance: Roll  {dist_mag:.2f} Nm")

            if curr_dist_p != prev['dist_p']:
                prev['dist_p'] = curr_dist_p
                dist_torque = np.array([0.0, dist_mag / base_env.max_torque, 0.0], dtype=np.float32)
                dist_count  = DIST_STEPS
                print(f"Disturbance: Pitch {dist_mag:.2f} Nm")

            if curr_dist_y != prev['dist_y']:
                prev['dist_y'] = curr_dist_y
                dist_torque = np.array([0.0, 0.0, dist_mag / base_env.max_torque], dtype=np.float32)
                dist_count  = DIST_STEPS
                print(f"Disturbance: Yaw   {dist_mag:.2f} Nm")

            if is_running:
                action, _ = model.predict(obs, deterministic=True)
                if dist_count > 0:
                    action[0] += dist_torque
                    dist_count -= 1
                torques = action[0] * base_env.max_torque * -1.0

                obs, _, done, _ = vec_env.step(action)
                raw_obs = base_env._get_obs()

                t_elapsed += base_env.dt
                buf_t.append(t_elapsed)
                buf_er.append(math.degrees(float(raw_obs[0])))
                buf_ep.append(math.degrees(float(raw_obs[1])))
                buf_ey.append(math.degrees(float(raw_obs[2])))
                buf_tx.append(float(torques[0]))
                buf_ty.append(float(torques[1]))
                buf_tz.append(float(torques[2]))

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

                if done[0]:
                    is_running = False
                    print(f"*** FALLEN *** at t={t_elapsed:.2f}s  Press 'Reset Robot' to restart.")
            else:
                time.sleep(base_env.dt)

    except KeyboardInterrupt:
        vec_env.close()
        plt.close()
