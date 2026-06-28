"""
各モデルに複数レベルの外乱を自動で与え、生存時間を計測してグラフ化する。
PyBullet DIRECT モード（GUI なし）で高速実行。
"""
import pybullet as p
import pybullet_data
import numpy as np
import math
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

MODELS = [
    ("Normal (6-dim)",    "models/PPO_Normal/final_model",    "models/PPO_Normal/vec_normalize.pkl",    6),
    ("Robust (6-dim)",    "models/PPO_Robust/final_model",    "models/PPO_Robust/vec_normalize.pkl",    6),
    ("Normal+WV (9-dim)", "models/PPO_Normal_WV/final_model", "models/PPO_Normal_WV/vec_normalize.pkl", 9),
    ("Robust+WV (9-dim)", "models/PPO_Robust_WV/final_model", "models/PPO_Robust_WV/vec_normalize.pkl", 9),
]

DIST_LEVELS  = [0.0, 0.3, 0.5, 0.8, 1.0, 1.5]   # [Nm]
TRIALS       = 5       # 試行回数（平均を取る）
MAX_STEPS    = 2400    # 最大ステップ数（= 10 秒）
DIST_STEPS   = 24      # 外乱印加ステップ数（= 0.1 秒）
DIST_START   = 480     # 外乱を与え始めるステップ（= 2 秒後）

TARGET_ROLL  = math.pi / 4.0
TARGET_PITCH = math.atan(1.0 / math.sqrt(2.0))
TARGET_YAW   = 0.0
HEIGHT       = 0.15 / math.sqrt(3)
MAX_TORQUE   = 0.5
DT           = 1.0 / 240.0


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def get_obs_6(boxId, client):
    _, quat    = p.getBasePositionAndOrientation(boxId, physicsClientId=client)
    roll, pitch, yaw = p.getEulerFromQuaternion(quat, physicsClientId=client)
    _, ang_vel = p.getBaseVelocity(boxId, physicsClientId=client)
    return np.array([
        _wrap(roll - TARGET_ROLL), _wrap(pitch - TARGET_PITCH), _wrap(yaw - TARGET_YAW),
        ang_vel[0], ang_vel[1], ang_vel[2],
    ], dtype=np.float32)


def get_obs_9(boxId, client):
    obs6 = get_obs_6(boxId, client)
    wvel = np.array([p.getJointState(boxId, i, physicsClientId=client)[1] for i in range(3)],
                    dtype=np.float32)
    return np.concatenate([obs6, wvel])


def run_trial(model, vec_env, obs_dim, dist_nm, client, boxId):
    ori = p.getQuaternionFromEuler([TARGET_ROLL, TARGET_PITCH, TARGET_YAW], physicsClientId=client)
    p.resetBasePositionAndOrientation(boxId, [0, 0, HEIGHT], ori, physicsClientId=client)
    p.resetBaseVelocity(boxId, [0, 0, 0], [0, 0, 0], physicsClientId=client)
    for i in range(3):
        p.resetJointState(boxId, i, 0, 0, physicsClientId=client)
        p.setJointMotorControl2(boxId, i, p.VELOCITY_CONTROL, force=0, physicsClientId=client)

    dist_torque = np.array([dist_nm / MAX_TORQUE, 0.0, 0.0], dtype=np.float32)
    dist_count  = 0

    get_obs = get_obs_9 if obs_dim == 9 else get_obs_6

    for step in range(MAX_STEPS):
        if step == DIST_START and dist_nm > 0:
            dist_count = DIST_STEPS

        raw_obs = get_obs(boxId, client)
        norm_obs = vec_env.normalize_obs(raw_obs.reshape(1, -1))
        action, _ = model.predict(norm_obs, deterministic=True)

        if dist_count > 0:
            action[0] += dist_torque
            dist_count -= 1

        torque = np.array(action).flatten() * MAX_TORQUE * -1.0
        for i, t in enumerate(torque):
            p.setJointMotorControl2(boxId, i, p.TORQUE_CONTROL, force=t, physicsClientId=client)
        p.stepSimulation(physicsClientId=client)

        if abs(raw_obs[0]) > 0.8 or abs(raw_obs[1]) > 0.8:
            return step * DT

    return MAX_STEPS * DT  # 生存


if __name__ == "__main__":
    from cubli_env import CubliEnv  # 6-dim 用
    from cubli_env_wv import CubliEnv as CubliEnvWV  # 9-dim 用

    results = {}  # label -> {dist_nm: mean_survival}

    for label, model_path, stats_path, obs_dim in MODELS:
        print(f"\n=== {label} ===")
        EnvClass = CubliEnvWV if obs_dim == 9 else CubliEnv
        dummy = EnvClass(render_mode=None)
        vec_env = VecNormalize.load(stats_path, DummyVecEnv([lambda: dummy]))
        vec_env.training = False
        vec_env.norm_reward = False
        model = PPO.load(model_path, env=vec_env)

        client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        p.loadURDF("plane.urdf", physicsClientId=client)
        boxId = p.loadURDF("day7/my_cubli.urdf",
                           [0, 0, HEIGHT],
                           p.getQuaternionFromEuler([TARGET_ROLL, TARGET_PITCH, TARGET_YAW],
                                                    physicsClientId=client),
                           physicsClientId=client)
        p.changeDynamics(boxId, -1, lateralFriction=1.0, spinningFriction=0.1, physicsClientId=client)
        for i in range(3):
            p.setJointMotorControl2(boxId, i, p.VELOCITY_CONTROL, force=0, physicsClientId=client)

        results[label] = {}
        for dist_nm in DIST_LEVELS:
            times = [run_trial(model, vec_env, obs_dim, dist_nm, client, boxId)
                     for _ in range(TRIALS)]
            mean_t = np.mean(times)
            results[label][dist_nm] = mean_t
            marker = " (MAX)" if mean_t >= MAX_STEPS * DT else ""
            print(f"  dist={dist_nm:.1f} Nm  survival={mean_t:.2f}s{marker}")

        p.disconnect(physicsClientId=client)
        dummy.close()

    # ─── プロット ───
    colors = ["tab:blue", "tab:red", "tab:cyan", "tab:orange"]
    markers = ["o", "s", "^", "D"]
    fig, ax = plt.subplots(figsize=(10, 5))
    fig.suptitle("Disturbance Tolerance Test\n(mean of 5 trials, disturbance at t=2s, max 10s)",
                 fontsize=12)

    for (label, *_), color, marker in zip(MODELS, colors, markers):
        if label not in results:
            continue
        xs = list(results[label].keys())
        ys = list(results[label].values())
        ax.plot(xs, ys, label=label, color=color, marker=marker,
                linewidth=2, markersize=8)

    ax.set_xlabel("Disturbance [Nm]")
    ax.set_ylabel("Survival Time [s]")
    ax.axhline(MAX_STEPS * DT, color="gray", linestyle=":", linewidth=1, label=f"Max ({MAX_STEPS*DT:.0f}s)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    out = "day7/disturbance_test.png"
    plt.tight_layout()
    plt.savefig(out, dpi=130)
    print(f"\n保存: {out}")
    plt.show()
