#!/usr/bin/env python3
"""
MuJoCo Gymnasium balance env for the two-wheeled bipedal (wheel-legged) robot.

Why MuJoCo: Gazebo Classic could not hold the load-bearing revolute leg joints (they
collapse the instant physics runs). MuJoCo's constrained dynamics + position actuators
hold them realistically, so the robot is a genuine inverted pendulum on its wheels and
the from-scratch PPO gets a real balance signal. No ROS, no display -> fast + headless.

Stage 1: the RL commands 2 wheel torques; the 8 leg joints are held at the 0 stance by
position actuators (still revolute + separately actuated -> a later stage can command
them to walk). Model: ../mujoco/mybot.xml (built from mybot_geometry.urdf.xacro).
"""
import os
from collections import deque

import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces

_HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.environ.get(
    "MJ_MODEL", os.path.abspath(os.path.join(_HERE, "..", "mujoco", "mybot.xml")))

WHEEL_JOINTS = ["Left_joint", "Right_joint"]
# 6 joints exposed in the observation (matches the Gazebo env's HIP_JOINTS set)
HIP_JOINTS = ["hip_yaw_left", "hip_roll_left", "Upper_left_joint",
              "hip_yaw_right", "hip_roll_right", "Upper_right_joint"]
# all 8 leg joints held at the 0 stance
LEG_JOINTS = ["hip_yaw_left", "hip_roll_left", "Upper_left_joint", "Lower_left_joint",
              "hip_yaw_right", "hip_roll_right", "Upper_right_joint", "Lower_right_joint"]


class MujocoBalanceEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, render=False, control_hz=50.0, max_steps=1000, seed=None):
        self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        self.sim_dt = float(self.model.opt.timestep)
        self.n_sub = max(1, int(round((1.0 / control_hz) / self.sim_dt)))
        self.max_steps = int(max_steps)

        # qpos/qvel addresses per joint
        self._q = {j: int(self.model.joint(j).qposadr[0]) for j in WHEEL_JOINTS + LEG_JOINTS}
        self._v = {j: int(self.model.joint(j).dofadr[0]) for j in WHEEL_JOINTS + LEG_JOINTS}
        # base free joint (the floating-base 6-DOF root): qpos[0:7]=[x y z qw qx qy qz], qvel[0:6]
        self._base_qadr, self._base_vadr = self._find_free_joint()
        # actuators: found by their target joint (robust to whatever names the model uses)
        self.wheel_act = [self._act_for_joint(j) for j in WHEEL_JOINTS]
        self.leg_act = [self._act_for_joint(j) for j in LEG_JOINTS]

        self.max_wheel_torque = 20.0
        self.k = 3                         # history stack
        self._frame_dim = 9 + len(HIP_JOINTS) * 2 + 2     # 23
        self.observation_space = spaces.Box(
            -np.inf, np.inf, (self._frame_dim * self.k,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (len(WHEEL_JOINTS),), np.float32)

        self._hist = deque(maxlen=self.k)
        self._last_action = np.zeros(len(WHEEL_JOINTS), np.float32)
        self.steps = 0
        self._rng = np.random.default_rng(seed)

        # reward shaping (mirrors the Gazebo balance env)
        self.w_tilt = 2.0
        self.w_drift = 0.05
        self.w_act = 0.01
        self.w_yaw = 0.15                  # penalize yaw-rate so it balances in place
                                           # instead of spinning/circling (wz already in obs)
        self.fall_tilt = 0.6               # rad; terminate past this
        self._qpos0 = self.model.qpos0.copy()
        self._spawn_z = float(self._qpos0[self._base_qadr + 2])

        self.render_mode = "human" if render else None
        self._viewer = None
        self._renderer = None

    # ---- model introspection -------------------------------------------------
    def _find_free_joint(self):
        for j in range(self.model.njnt):
            if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                return int(self.model.jnt_qposadr[j]), int(self.model.jnt_dofadr[j])
        raise RuntimeError("no free (floating-base) joint in the model")

    def _act_for_joint(self, jname):
        jid = int(self.model.joint(jname).id)
        for a in range(self.model.nu):
            if int(self.model.actuator_trnid[a, 0]) == jid:
                return a
        raise ValueError(f"no actuator targets joint '{jname}'")

    # ---- observation ---------------------------------------------------------
    def _base_rpy_tilt(self):
        quat = self.data.qpos[self._base_qadr + 3: self._base_qadr + 7]   # w x y z
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], np.hypot(R[2, 1], R[2, 2]))
        tilt = np.arccos(np.clip(R[2, 2], -1.0, 1.0))    # angle of body-z from world-z
        return float(roll), float(pitch), float(tilt)

    def _read_frame(self):
        roll, pitch, _ = self._base_rpy_tilt()
        wx, wy, wz = self.data.qvel[self._base_vadr + 3: self._base_vadr + 6]     # body ang vel
        bvx, bvy = self.data.qvel[self._base_vadr + 0], self.data.qvel[self._base_vadr + 1]
        wl = self.data.qvel[self._v["Left_joint"]]
        wr = self.data.qvel[self._v["Right_joint"]]
        hip_pos = [self.data.qpos[self._q[j]] for j in HIP_JOINTS]
        hip_vel = [self.data.qvel[self._v[j]] for j in HIP_JOINTS]
        frame = np.array([roll, pitch, wx, wy, wz, wl, wr, bvx, bvy]
                         + hip_pos + hip_vel + list(self._last_action), dtype=np.float32)
        return np.clip(np.nan_to_num(frame), -50.0, 50.0)

    def _obs(self):
        return np.concatenate(list(self._hist), dtype=np.float32)

    def _hold_legs_ctrl(self):
        for a in self.leg_act:
            self.data.ctrl[a] = 0.0        # position actuators -> hold the 0 stance

    # ---- gym API -------------------------------------------------------------
    def reset(self, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self._qpos0
        # small random upright perturbation for robustness
        droll = float(self._rng.uniform(-0.05, 0.05))
        dpitch = float(self._rng.uniform(-0.05, 0.05))
        quat = np.zeros(4)
        mujoco.mju_euler2Quat(quat, np.array([droll, dpitch, 0.0]), "xyz")
        self.data.qpos[self._base_qadr + 3: self._base_qadr + 7] = quat
        self.data.qpos[self._base_qadr + 2] = self._spawn_z
        self.data.qvel[:] = 0.0
        self._last_action[:] = 0.0
        self._hold_legs_ctrl()
        for a in self.wheel_act:
            self.data.ctrl[a] = 0.0
        mujoco.mj_forward(self.model, self.data)
        # settle a couple of control steps with the legs held
        for _ in range(2):
            self._hold_legs_ctrl()
            for _ in range(self.n_sub):
                mujoco.mj_step(self.model, self.data)
        self.steps = 0
        f = self._read_frame()
        self._hist.clear()
        for _ in range(self.k):
            self._hist.append(f)
        return self._obs(), {}

    def step(self, action):
        action = np.asarray(action, np.float32).reshape(-1)
        a = np.clip(action, -1.0, 1.0)
        self._last_action = a
        tau = a * self.max_wheel_torque
        for i, act in enumerate(self.wheel_act):
            self.data.ctrl[act] = float(tau[i])
        self._hold_legs_ctrl()
        for _ in range(self.n_sub):
            mujoco.mj_step(self.model, self.data)

        self._hist.append(self._read_frame())
        self.steps += 1
        roll, pitch, tilt = self._base_rpy_tilt()
        base_x = self.data.qpos[self._base_qadr + 0]
        base_y = self.data.qpos[self._base_qadr + 1]
        drift2 = base_x * base_x + base_y * base_y
        wz = float(self.data.qvel[self._base_vadr + 5])     # yaw rate (body-z ang vel)
        reward = (1.0
                  - self.w_tilt * tilt * tilt
                  - self.w_drift * drift2
                  - self.w_yaw * wz * wz
                  - self.w_act * float(np.sum(a * a)))
        terminated = bool(tilt > self.fall_tilt)
        truncated = bool(self.steps >= self.max_steps)
        if terminated:
            reward -= 5.0
        if self.render_mode == "human":
            self.render()
        return self._obs(), float(reward), terminated, truncated, {"tilt": tilt}

    # ---- rendering -----------------------------------------------------------
    def render(self):
        if self.render_mode == "human":
            if self._viewer is None:
                from mujoco import viewer as _mjviewer   # NB: don't `import mujoco.viewer`
                self._viewer = _mjviewer.launch_passive(self.model, self.data)  # (rebinds `mujoco` local)
            self._viewer.sync()
            return None
        # rgb_array (offscreen; no display needed)
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=720, width=1280)
            # prefer the model's external "track" camera (frames the robot) over the free cam
            try:
                self._cam = int(self.model.camera("track").id)
            except Exception:
                self._cam = -1
        self._renderer.update_scene(self.data, camera=self._cam)
        return self._renderer.render()

    def close(self):
        if self._viewer is not None:
            self._viewer.close(); self._viewer = None
        if self._renderer is not None:
            self._renderer.close(); self._renderer = None
