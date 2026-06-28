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

import argparse
import pybullet as p
import math
import time
import os

from collections import deque

import numpy as np
import matplotlib.pyplot as plt

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from cubli_env_wv import CubliEnv
#from types import SimpleNamespace

PLOT_LEN   = 480
PLOT_EVERY = 100
DIST_STEPS = 24  # 外乱の継続ステップ数


def parse_args():
    parser = argparse.ArgumentParser(description="Cubli PPO 評価スクリプト")
    parser.add_argument(
        "--model", choices=["normal", "robust"], default="robust",
        help="評価するモデル種別 (default: robust)",
    )
    parser.add_argument(
        "--model-dir", default=None,
        help="モデルディレクトリを直接指定する場合 (例: models/PPO_Robust)",
    )
    return parser.parse_args()


def resolve_paths(args):
    if args.model_dir:
        base = args.model_dir
        title = os.path.basename(base)
        max_dist = 2.5
    elif args.model == "robust":
        base     = "models/PPO_Robust_WV"
        title    = "PPO Robust"
        max_dist = 2.5
    else:
        base     = "models/PPO_Normal_WV"
        title    = "PPO Normal"
        max_dist = 2.0

    return (
        f"{base}/final_model",
        f"{base}/vec_normalize.pkl",
        title,
        max_dist,
    )


def setup_buttons(client, max_dist):
    btns = {
        "start":    p.addUserDebugParameter(">>> START PPO <<<",       1,        0,        0,        physicsClientId=client),
        "reset":    p.addUserDebugParameter("Reset Robot",             1,        0,        0,        physicsClientId=client),
        "dist_r":   p.addUserDebugParameter("Disturbance: Roll",       1,        0,        0,        physicsClientId=client),
        "dist_p":   p.addUserDebugParameter("Disturbance: Pitch",      1,        0,        0,        physicsClientId=client),
        "dist_y":   p.addUserDebugParameter("Disturbance: Yaw",        1,        0,        0,        physicsClientId=client),
        "dist_mag": p.addUserDebugParameter("Disturbance Torque [Nm]", 0, max_dist, 0.5,   physicsClientId=client),
    }
    prev = {k: p.readUserDebugParameter(v, physicsClientId=client) for k, v in btns.items()}
    return btns, prev


def build_plot(title):
#⑫ build_plot() の返り値が 9 個でアンパックが冗長
#  g076a:109、g076a:172：9値タプルのアンパックは可読性が低い。
#  SimpleNamespace にまとめると呼び出し側がすっきりする：
#def build_plot(title) -> SimpleNamespace:
    plt.ion()
    fig = plt.figure(figsize=(10, 6))
    gs  = plt.GridSpec(2, 1, hspace=0.4)
    fig.suptitle(title)

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
    return (
        fig, ax_err, ax_trq, line_er, line_ep, line_ey, line_tx, line_ty, line_tz
    )


def update_plot(fig, axes, lines, bufs):
    ax_err, ax_trq = axes
    line_er, line_ep, line_ey, line_tx, line_ty, line_tz = lines
    buf_t, buf_er, buf_ep, buf_ey, buf_tx, buf_ty, buf_tz = bufs

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

def main():
    args = parse_args()
    model_path, stats_path, title, max_dist = resolve_paths(args)

    if not os.path.exists(model_path + ".zip"):
        print(f"モデルが見つかりません: {model_path}.zip")
        print("先に train_normal.py または train_robust.py を実行してください。")
        return
    if not os.path.exists(stats_path):
        print(f"正規化パラメータが見つかりません: {stats_path}")
        return

    base_env = CubliEnv(render_mode="human")
    client   = base_env.physicsClient

    vec_env = DummyVecEnv([lambda: base_env])
    vec_env = VecNormalize.load(stats_path, vec_env)
    vec_env.training    = False
    vec_env.norm_reward = False

    model = PPO.load(model_path, env=vec_env)
    obs   = vec_env.reset()
    p.resetDebugVisualizerCamera(0.4, 45, -20, [0, 0, 0.1], physicsClientId=client)

    btns, prev = setup_buttons(client, max_dist)

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
    bufs = (buf_t, buf_er, buf_ep, buf_ey, buf_tx, buf_ty, buf_tz)

    #plot = build_plot(title)
    fig, ax_err, ax_trq, line_er, line_ep, line_ey, line_tx, line_ty, line_tz = build_plot(title)
    axes  = (ax_err, ax_trq)
    lines = (line_er, line_ep, line_ey, line_tx, line_ty, line_tz)

    print(f"Ready ({title}). Press '>>> START PPO <<<' to begin.")

    try:
        while True:
            curr_start  = p.readUserDebugParameter(btns["start"],    physicsClientId=client)
            curr_reset  = p.readUserDebugParameter(btns["reset"],    physicsClientId=client)
            curr_dist_r = p.readUserDebugParameter(btns["dist_r"],   physicsClientId=client)
            curr_dist_p = p.readUserDebugParameter(btns["dist_p"],   physicsClientId=client)
            curr_dist_y = p.readUserDebugParameter(btns["dist_y"],   physicsClientId=client)
            dist_mag    = p.readUserDebugParameter(btns["dist_mag"], physicsClientId=client)

            if curr_start != prev["start"]:
                prev["start"] = curr_start
                is_running = True
                print("PPO Started!")

            if curr_reset != prev["reset"]:
                is_running = False
                obs = vec_env.reset()
                p.resetDebugVisualizerCamera(0.4, 45, -20, [0, 0, 0.1], physicsClientId=client)
                for v in btns.values():
                    p.removeUserDebugItem(v, physicsClientId=client)
                btns, prev = setup_buttons(client, max_dist)
                t_elapsed  = 0.0
                step_count = 0
                dist_count = 0
                for buf in bufs:
                    buf.clear()
                print("Reset & Paused.")
                continue

            if curr_dist_r != prev["dist_r"]:
                prev["dist_r"] = curr_dist_r
                dist_torque = np.array([dist_mag / base_env.max_torque, 0.0, 0.0], dtype=np.float32)
                dist_count  = DIST_STEPS
                print(f"Disturbance: Roll  {dist_mag:.2f} Nm")

            if curr_dist_p != prev["dist_p"]:
                prev["dist_p"] = curr_dist_p
                dist_torque = np.array([0.0, dist_mag / base_env.max_torque, 0.0], dtype=np.float32)
                dist_count  = DIST_STEPS
                print(f"Disturbance: Pitch {dist_mag:.2f} Nm")

            if curr_dist_y != prev["dist_y"]:
                prev["dist_y"] = curr_dist_y
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
                # 現状（プライベートメソッド外部呼び出し）
                #raw_obs = base_env._get_obs()
                # 修正（g071 に get_obs() 追加後）
                raw_obs = base_env.get_obs()
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
                    update_plot(fig, axes, lines, bufs)

                if done[0]:
                    is_running = False
                    print(f"*** FALLEN *** at t={t_elapsed:.2f}s  Press 'Reset Robot' to restart.")
            else:
                time.sleep(base_env.dt)

    except KeyboardInterrupt:
        pass
    finally:
        vec_env.close()
        plt.close()


if __name__ == "__main__":
    main()

# g076a_enjoy.py の使用法を説明します。
#  ---
#  ■ 前提条件
#  実行前に、いずれかの学習スクリプトを完了させておく必要があります。
#  python g072_train_normal.py   # Normal モデルを学習
#  python g073_train_robust.py   # Robust モデルを学習
#  ■ 学習後に以下のファイルが生成されていることを確認してください。
#  models/PPO_Normal/final_model.zip
#  models/PPO_Normal/vec_normalize.pkl
#  models/PPO_Robust/final_model.zip
#  models/PPO_Robust/vec_normalize.pkl
#  ---
#  ■ 起動方法
#  ▼ パターン1: Robust モデルで起動（デフォルト）
#  python g076a_enjoy.py
#  python g076a_enjoy.py --model robust
#  ▼ パターン2: Normal モデルで起動
#  python g076a_enjoy.py --model normal
#  ▼パターン3: モデルディレクトリを直接指定
#  チェックポイントなど任意のディレクトリを指定できます。
#  python g076a_enjoy.py --model-dir models/PPO_Robust
#  python g076a_enjoy.py --model-dir models_1angle_penalty-100/PPO_Robust
#  ---
#  ■ 起動後の操作
#  PyBullet GUI ウィンドウと matplotlib グラフウィンドウの2つが開きます。
#  PyBullet GUI のボタン
#  ┌─────────────────────────┬────────────┬──────────────────────────────────────────────────────────┐
#  │         ボタン          │    操作    │                           効果                           │
#  ├─────────────────────────┼────────────┼──────────────────────────────────────────────────────────┤
#  │ >>> START PPO <<<       │ クリック   │ PPO 制御を開始。ロボットが自律バランスを始める           │
#  ├─────────────────────────┼────────────┼──────────────────────────────────────────────────────────┤
#  │ Reset Robot             │ クリック   │ 制御を停止し、ロボットを初期姿勢に戻す。グラフもクリア   │
#  ├─────────────────────────┼────────────┼──────────────────────────────────────────────────────────┤
#  │ Disturbance: Roll       │ クリック   │ Roll 方向に外乱トルクを 24 ステップ（0.1 秒）印加        │
#  ├─────────────────────────┼────────────┼──────────────────────────────────────────────────────────┤
#  │ Disturbance: Pitch      │ クリック   │ Pitch 方向に外乱トルクを 24 ステップ印加                 │
#  ├─────────────────────────┼────────────┼──────────────────────────────────────────────────────────┤
#  │ Disturbance: Yaw        │ クリック   │ Yaw 方向に外乱トルクを 24 ステップ印加                   │
#  ├─────────────────────────┼────────────┼──────────────────────────────────────────────────────────┤
#  │ Disturbance Torque [Nm] │ スライダー │ 外乱の強さを 0〜2.5 Nm の範囲で設定（デフォルト 0.5 Nm） │
#  └─────────────────────────┴────────────┴──────────────────────────────────────────────────────────┘
#  ■ 操作手順
#  1. 起動 → "Ready (PPO Robust). Press '>>> START PPO <<<' to begin." と表示される
#  2. [>>> START PPO <<<] をクリック → バランス制御が始まる
#  3. スライダーで外乱強度を設定（例: 1.0 Nm）
#  4. [Disturbance: Roll] などをクリック → 外乱を与えてロボットの応答を観察
#  5. 転倒したら → "*** FALLEN *** at t=XX.XXs  Press 'Reset Robot' to restart."
#  6. [Reset Robot] でリスタート
#  7. Ctrl+C で終了
#  ---
#  ■ matplotlib グラフの見方
#  100 ステップごと（約 0.42 秒ごと）に更新される2段グラフです。
#  ┌─────────────────────────┬─────────────────────────────────────────────────────────────┬──────────────────────┐
#  │         グラフ          │                            内容                             │         目標         │
#  ├─────────────────────────┼─────────────────────────────────────────────────────────────┼──────────────────────┤
#  │ 上段: Angle Error [deg] │ Roll（赤）・Pitch（緑）・Yaw（青）の目標角度からの誤差      │ すべて 0° 付近に収束 │
#  ├─────────────────────────┼─────────────────────────────────────────────────────────────┼──────────────────────┤
#  │ 下段: Motor Torque [Nm] │ joint_x（赤）・joint_y（緑）・joint_z（青）のモータートルク │ 直立時は 0 Nm 付近   │
#  └─────────────────────────┴─────────────────────────────────────────────────────────────┴──────────────────────┘
#  外乱を与えたとき、上段でスパイクが発生し、その直後に下段でカウンタートルクが出れば「外乱に反応できている」状態です。
#  ---
#  ■ 終了方法
#  ターミナルで Ctrl+C を押すと finally ブロックが実行され、PyBullet と matplotlib が確実にクローズされます。