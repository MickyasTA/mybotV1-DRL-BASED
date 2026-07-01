#!/usr/bin/env python3
"""
Stage 2 — goal-point navigation with ACTUATED legs (MuJoCo, from-scratch PPO).

Builds on the balance env: the robot must drive to a randomized goal point on flat
ground and STOP there (station-keep), while staying upright. Two things change vs
Stage 1 balancing:

  * Action grows 2 -> 10: 2 wheel torques + 8 leg position targets (the legs are no
    longer locked; the policy commands them around the 0 stance via the PD servos).
  * Observation gains a base-frame GOAL vector (dir + heading error + distance) and now
    exposes all 8 leg joints (the balance obs only had the 6 hips).

Station-keeping at the goal is what removes the balancer's residual slow backward
drift: "stay put" is only rewarded once you are AT the goal; everywhere else the robot
must make progress toward it, so standing still (or drifting) is penalized.

Goal representation & reward follow wheeled-legged goal-nav practice (base-frame goal +
potential-based progress + velocity-toward-goal + face-goal + arrival/station-keep),
e.g. ETH/RSL wheeled-legged stair-climbing (arXiv 2402.06143) and MuJoCo Playground
(arXiv 2502.08844). Verification of those sources was rate-limited; treat weights as
tunable starting points.
"""
import math
import numpy as np
import mujoco
from gymnasium import spaces

from balance_env_mujoco import MujocoBalanceEnv, WHEEL_JOINTS, LEG_JOINTS


class GoalNavEnv(MujocoBalanceEnv):
    def __init__(self, render=False, control_hz=50.0, max_steps=1000, seed=None,
                 goal_reward=True):
        super().__init__(render=render, control_hz=control_hz, max_steps=max_steps, seed=seed)
        # curriculum: goal_reward=False -> Stage 2a "balance WITH actuated legs" (no goal
        # terms; station-keep at origin via a drift penalty), same obs/action as 2b so the
        # goal stage can warm-start from it. goal_reward=True -> Stage 2b "drive to goal".
        self.goal_reward = goal_reward
        self.w_drift = 0.05

        self.n_wheel = len(WHEEL_JOINTS)          # 2
        self.n_leg = len(LEG_JOINTS)              # 8
        self.act_dim = self.n_wheel + self.n_leg  # 10
        self.leg_scale = 0.25                      # rad; leg target = leg_scale * a_leg (around 0 stance)
                                                   # (small so exploration jitter doesn't topple balance;
                                                   # widen at the ducking/terrain stage where legs matter)

        # action: [wheelL, wheelR, 8 leg position targets], all in [-1, 1]
        self.action_space = spaces.Box(-1.0, 1.0, (self.act_dim,), np.float32)
        self._last_action = np.zeros(self.act_dim, np.float32)

        # obs per frame: base(9) + all-8-leg pos(8)+vel(8) + goal(4) + last_action(10) = 39
        self._frame_dim = 9 + self.n_leg * 2 + 4 + self.act_dim
        self.observation_space = spaces.Box(
            -np.inf, np.inf, (self._frame_dim * self.k,), np.float32)

        # goal state
        self.goal_min_r = 0.8
        self.goal_max_r = 2.0
        self.goal_tol = 0.25                       # m; "arrived" radius
        self.goal_xy = np.zeros(2, np.float32)
        self._prev_dist = 0.0
        try:
            self._goal_mocap = int(self.model.body("goal_marker").mocapid[0])
        except Exception:
            self._goal_mocap = -1

        # reward weights (balance terms inherited: w_tilt, w_yaw; drift penalty dropped for nav)
        self.w_prog = 10.0      # potential-based progress toward goal (dense)
        self.w_vel = 0.3        # velocity projected onto goal direction
        self.w_face = 0.3       # face the goal (cos of heading error)
        self.w_arrive = 5.0     # bonus while inside goal_tol
        self.w_stop = 1.0       # station-keep: penalize speed once at goal (kills drift)
        self.w_stall = 0.5      # penalize standing still while far from goal
        self.w_arate = 0.02     # action-rate smoothness
        self.w_act = 0.005      # override: lower action penalty (10-d action now)
        self.w_pose = 0.6       # keep legs near the nominal (0) stance -> no cheap
                                # leg-splaying for balance; legs move only when it pays
                                # off (later: ducking/terrain). nominal leg qpos = 0.

    # ---- goal geometry -------------------------------------------------------
    def _base_yaw(self):
        w, x, y, z = self.data.qpos[self._base_qadr + 3: self._base_qadr + 7]
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _goal_features(self):
        bx = float(self.data.qpos[self._base_qadr + 0])
        by = float(self.data.qpos[self._base_qadr + 1])
        dx, dy = float(self.goal_xy[0]) - bx, float(self.goal_xy[1]) - by
        dist = math.hypot(dx, dy)
        yaw = self._base_yaw()
        c, s = math.cos(-yaw), math.sin(-yaw)     # rotate world delta into base frame
        bxr = c * dx - s * dy
        byr = s * dx + c * dy
        heading_err = math.atan2(byr, bxr)         # 0 => goal straight ahead
        n = max(dist, 1e-6)
        return np.array([bxr / n, byr / n], np.float32), heading_err, dist

    # ---- observation ---------------------------------------------------------
    def _read_frame(self):
        roll, pitch, _ = self._base_rpy_tilt()
        wx, wy, wz = self.data.qvel[self._base_vadr + 3: self._base_vadr + 6]
        bvx, bvy = self.data.qvel[self._base_vadr + 0], self.data.qvel[self._base_vadr + 1]
        wl = self.data.qvel[self._v["Left_joint"]]
        wr = self.data.qvel[self._v["Right_joint"]]
        leg_pos = [self.data.qpos[self._q[j]] for j in LEG_JOINTS]     # all 8
        leg_vel = [self.data.qvel[self._v[j]] for j in LEG_JOINTS]     # all 8
        gdir, gher, gdist = self._goal_features()
        frame = np.array([roll, pitch, wx, wy, wz, wl, wr, bvx, bvy]
                         + leg_pos + leg_vel
                         + [float(gdir[0]), float(gdir[1]), float(gher), float(gdist)]
                         + list(self._last_action), dtype=np.float32)
        return np.clip(np.nan_to_num(frame), -50.0, 50.0)

    # ---- gym API -------------------------------------------------------------
    def _sample_goal(self):
        ang = float(self._rng.uniform(-math.pi, math.pi))
        r = float(self._rng.uniform(self.goal_min_r, self.goal_max_r))
        self.goal_xy = np.array([r * math.cos(ang), r * math.sin(ang)], np.float32)

    def reset(self, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._sample_goal()                        # must precede super().reset() (obs reads goal)
        obs, info = super().reset(seed=seed, options=options)
        if self._goal_mocap >= 0:
            self.data.mocap_pos[self._goal_mocap] = [self.goal_xy[0], self.goal_xy[1], 0.1]
        self._prev_dist = float(math.hypot(self.goal_xy[0], self.goal_xy[1]))  # base spawns at origin
        return obs, info

    def _apply_action(self, a):
        tau = a[:self.n_wheel] * self.max_wheel_torque
        for i, act in enumerate(self.wheel_act):
            self.data.ctrl[act] = float(tau[i])
        a_leg = a[self.n_wheel:]
        for i, act in enumerate(self.leg_act):
            lo, hi = self.model.actuator_ctrlrange[act]
            self.data.ctrl[act] = float(np.clip(self.leg_scale * a_leg[i], lo, hi))

    def step(self, action):
        action = np.asarray(action, np.float32).reshape(-1)
        a = np.clip(action, -1.0, 1.0)
        prev_a = self._last_action
        self._last_action = a
        self._apply_action(a)
        for _ in range(self.n_sub):
            mujoco.mj_step(self.model, self.data)

        self._hist.append(self._read_frame())
        self.steps += 1

        roll, pitch, tilt = self._base_rpy_tilt()
        wz = float(self.data.qvel[self._base_vadr + 5])
        bvx = float(self.data.qvel[self._base_vadr + 0])
        bvy = float(self.data.qvel[self._base_vadr + 1])
        speed = math.hypot(bvx, bvy)
        _, heading_err, dist = self._goal_features()
        progress = self._prev_dist - dist          # >0 when getting closer
        self._prev_dist = dist
        at_goal = dist < self.goal_tol
        # velocity projected onto goal direction (world frame)
        gx, gy = float(self.goal_xy[0]), float(self.goal_xy[1])
        bx = float(self.data.qpos[self._base_qadr + 0])
        by = float(self.data.qpos[self._base_qadr + 1])
        ddx, ddy = gx - bx, gy - by
        dn = max(math.hypot(ddx, ddy), 1e-6)
        v_to_goal = (ddx * bvx + ddy * bvy) / dn
        # posture: sum of squared leg-joint deviation from the 0 stance (kills leg-splaying)
        leg_dev = float(sum(self.data.qpos[self._q[j]] ** 2 for j in LEG_JOINTS))

        reward = (1.0                                             # alive
                  - self.w_tilt * tilt * tilt                     # stay upright
                  - self.w_yaw * wz * wz                          # no spinning
                  - self.w_pose * leg_dev                         # keep legs at stance (no splits)
                  - self.w_act * float(np.sum(a * a))             # effort
                  - self.w_arate * float(np.sum((a - prev_a) ** 2)))
        if self.goal_reward:                                      # Stage 2b: drive to goal
            reward += (self.w_prog * progress                     # get closer (potential-based)
                       + self.w_vel * float(np.clip(v_to_goal, -1.5, 1.5))
                       + self.w_face * math.cos(heading_err))     # face the goal
            if at_goal:
                reward += self.w_arrive - self.w_stop * speed * speed   # station-keep at goal
            elif speed < 0.1 and dist > 0.5:
                reward -= self.w_stall                            # don't just stand there
        else:                                                     # Stage 2a: balance in place
            reward -= self.w_drift * (bx * bx + by * by)          # station-keep at origin

        terminated = bool(tilt > self.fall_tilt)
        truncated = bool(self.steps >= self.max_steps)
        if terminated:
            reward -= 5.0
        if self.render_mode == "human":
            self.render()
        return self._obs(), float(reward), terminated, truncated, {
            "tilt": tilt, "dist": dist, "at_goal": at_goal}
