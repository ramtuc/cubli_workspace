import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import math
import time


class CubliEnv(gym.Env):
    def __init__(self, render_mode=None):
        super().__init__()
        self.render_mode = render_mode

        if render_mode == "human":
            self.physicsClient = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(
                cameraDistance=0.4, cameraYaw=45,
                cameraPitch=-15, cameraTargetPosition=[0, 0, 0.1],
                physicsClientId=self.physicsClient,
            )
        else:
            self.physicsClient = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath(),
                                  physicsClientId=self.physicsClient)

        # 行動空間: 3軸トルク [-1.0 ~ 1.0]（実際のトルクは × max_torque）
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        # 観測空間: [角度誤差(3), 角速度(3)]
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)

        self.dt         = 1. / 240.
        self.max_torque = 0.5  # [Nm]

        # 頂点倒立の目標角度（Day 6 と同じ値）
        self.target_roll  = math.pi / 4.0
        self.target_pitch = math.atan(1.0 / math.sqrt(2.0))  # ≈ 35.26°
        self.target_yaw   = 0.0

        self.boxId              = None
        self.episode_error_sum  = 0.0
        self.episode_energy_sum = 0.0
        self.episode_steps      = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        cid = self.physicsClient
        p.resetSimulation(physicsClientId=cid)
        p.setGravity(0, 0, -9.81, physicsClientId=cid)
        if self.render_mode == "human":
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=cid)
        p.loadURDF("plane.urdf", physicsClientId=cid)

        # 初期姿勢にランダムノイズ（ドメインランダマイゼーション）
        rng   = np.random.default_rng(seed)
        noise = rng.uniform(-0.1, 0.1, 3)
        start_r = self.target_roll  + noise[0]
        start_p = self.target_pitch + noise[1]
        start_y = self.target_yaw   + noise[2]

        startPos         = [0, 0, 0.15 / math.sqrt(3)]
        startOrientation = p.getQuaternionFromEuler([start_r, start_p, start_y],
                                                    physicsClientId=cid)

        self.boxId = p.loadURDF("day7/my_cubli.urdf", startPos, startOrientation,
                                physicsClientId=cid)
        p.changeDynamics(self.boxId, -1, lateralFriction=1.0, spinningFriction=0.1,
                         physicsClientId=cid)

        for i in range(3):
            p.setJointMotorControl2(self.boxId, i, p.VELOCITY_CONTROL, force=0,
                                    physicsClientId=cid)

        self.episode_error_sum  = 0.0
        self.episode_energy_sum = 0.0
        self.episode_steps      = 0

        return self._get_obs(), {}

    def step(self, action):
        cid    = self.physicsClient
        torque = np.array(action) * self.max_torque * -1.0

        p.setJointMotorControl2(self.boxId, 0, p.TORQUE_CONTROL, force=torque[0], physicsClientId=cid)
        p.setJointMotorControl2(self.boxId, 1, p.TORQUE_CONTROL, force=torque[1], physicsClientId=cid)
        p.setJointMotorControl2(self.boxId, 2, p.TORQUE_CONTROL, force=torque[2], physicsClientId=cid)

        p.stepSimulation(physicsClientId=cid)
        if self.render_mode == "human":
            time.sleep(self.dt)

        obs = self._get_obs()
        err_r, err_p, err_y, vel_r, vel_p, vel_y = obs

        angle_penalty  = 10.0 * (err_r**2 + err_p**2 + err_y**2)
        vel_penalty    =  0.1 * (vel_r**2 + vel_p**2 + vel_y**2)
        action_penalty =  0.01 * np.sum(np.square(action))
        reward = 1.0 - angle_penalty - vel_penalty - action_penalty

        self.episode_error_sum  += (abs(err_r) + abs(err_p)) * 180 / math.pi
        self.episode_energy_sum += np.sum(np.abs(torque))
        self.episode_steps      += 1

        terminated = bool(abs(err_r) > 0.8 or abs(err_p) > 0.8)
        truncated  = False

        info = {}
        if terminated or truncated:
            info["episode_metrics"] = {
                "avg_angle_error_deg": self.episode_error_sum / max(self.episode_steps, 1),
                "total_energy":        self.episode_energy_sum,
                "survival_time":       self.episode_steps * self.dt,
            }

        return obs, reward, terminated, truncated, info

    def _get_obs(self):
        cid = self.physicsClient
        _, quat   = p.getBasePositionAndOrientation(self.boxId, physicsClientId=cid)
        roll, pitch, yaw = p.getEulerFromQuaternion(quat, physicsClientId=cid)
        _, ang_vel = p.getBaseVelocity(self.boxId, physicsClientId=cid)

        err_r = self._wrap(roll  - self.target_roll)
        err_p = self._wrap(pitch - self.target_pitch)
        err_y = self._wrap(yaw   - self.target_yaw)

        return np.array([err_r, err_p, err_y,
                         ang_vel[0], ang_vel[1], ang_vel[2]], dtype=np.float32)

    @staticmethod
    def _wrap(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def close(self):
        p.disconnect(physicsClientId=self.physicsClient)
