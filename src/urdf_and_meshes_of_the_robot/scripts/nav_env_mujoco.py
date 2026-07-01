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
                 goal_reward=True, model_path=None):
        super().__init__(render=render, control_hz=control_hz, max_steps=max_steps,
                         seed=seed, model_path=model_path)
        # curriculum: goal_reward=False -> Stage 2a "balance WITH actuated legs" (no goal
        # terms; station-keep at origin via a drift penalty), same obs/action as 2b so the
        # goal stage can warm-start from it. goal_reward=True -> Stage 2b "drive to goal".
        self.goal_reward = goal_reward
        self.w_drift = 0.05

        self.n_wheel = len(WHEEL_JOINTS)          # 2
        self.n_leg = len(LEG_JOINTS)              # 8
        self.act_dim = self.n_wheel + self.n_leg  # 10
        # per-joint action range (rad): leg target = leg_scale[i] * a_leg[i] around the
        # 0 stance. Small so exploration jitter doesn't topple balance; later stages
        # widen ONLY the squat (pitch) joints -- see ObstacleNavEnv.
        self.leg_scale = np.full(self.n_leg, 0.25)

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
        # anti-SCISSOR: the policy discovered a fore-aft split (one wheel pushed
        # forward, one back -> a static tripod, no balancing needed). Penalize the
        # wheelbase measured ALONG THE HEADING: ~0 in a proper stance and in symmetric
        # squats (both wheels move together), large for the scissor -> makes the cheat
        # decisively unprofitable without locking or restricting the legs.
        self.w_scissor = 10.0
        self.w_legvel = 0.002   # gentle "don't flail the legs" (weird jitter) term
        self._wheel_bid = [int(self.model.body("Left_wheel").id),
                           int(self.model.body("Right_wheel").id)]

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
            self.data.ctrl[act] = float(np.clip(self.leg_scale[i] * a_leg[i], lo, hi))

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
        # anti-scissor: fore-aft wheelbase along the heading (see __init__)
        pl = self.data.xpos[self._wheel_bid[0]]
        pr = self.data.xpos[self._wheel_bid[1]]
        yaw = self._base_yaw()
        d_fore = float((pl[0] - pr[0]) * math.cos(yaw) + (pl[1] - pr[1]) * math.sin(yaw))
        leg_vel2 = float(sum(self.data.qvel[self._v[j]] ** 2 for j in LEG_JOINTS))

        reward = (1.0                                             # alive
                  - self.w_tilt * tilt * tilt                     # stay upright
                  - self.w_yaw * wz * wz                          # no spinning
                  - self.w_pose * leg_dev                         # keep legs at stance (no splits)
                  - self.w_scissor * d_fore * d_fore              # no fore-aft scissor stance
                  - self.w_legvel * leg_vel2                      # no leg flailing
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
            "tilt": tilt, "dist": dist, "at_goal": at_goal, "scissor": abs(d_fore)}


# ============================================================================ #
#  Stage 3 — obstacle avoidance / dodging + DUCKING under a low table          #
# ============================================================================ #
class ObstacleNavEnv(GoalNavEnv):
    """Goal-nav plus obstacles. The scene gains mocap pillars and a low, wide table
    slab (mybot.xml, buried by default; placed here per reset). Perception is a cheap
    egocentric 2D lidar via mujoco.mj_ray with a geomgroup filter that sees ONLY the
    obstacle/terrain geoms (group 2): one LOW fan (pillar height), one HIGH fan (table-
    slab height), plus one UP ray (ceiling clearance) and the base height. Reward adds
    collision + proximity penalties and a height-target term that makes the robot SQUAT
    (legs bend -> base drops) to pass under the slab; the leg-posture penalty is relaxed
    while under it so ducking isn't fought by the stance regularizer.

    Frame layout = the 39-d Stage-2 frame as PREFIX + [12 low rays, 8 high rays, up ray,
    base_z] appended -> 61 per frame, 183 with the 3-frame stack. Keeping the old frame
    as a prefix is what lets ppo_parallel.py warm-start from a Stage-2 checkpoint by
    weight surgery (old first-layer columns copy over, new ray columns start at zero).
    """
    N_RAY_LOW = 12                 # 30 deg apart, sees pillars
    N_RAY_HIGH = 8                 # 45 deg apart, sees the table slab edge
    RAY_MAX = 4.0                  # m, horizontal rays
    UP_MAX = 1.2                   # m, ceiling-clearance ray

    def __init__(self, render=False, control_hz=50.0, max_steps=1000, seed=None,
                 p_table=0.5, n_pillars=(1, 4), model_path=None):
        super().__init__(render=render, control_hz=control_hz, max_steps=max_steps,
                         seed=seed, goal_reward=True, model_path=model_path)
        # Ducking = SQUATTING, not splaying: widen ONLY the pitch joints (Upper/knee,
        # which fold the legs under the body and lower the base straight down); hip
        # yaw/roll stay at the tight Stage-2 range so the lateral-splits shortcut
        # (which also lowers the body but is unstable and not the desired behavior)
        # remains physically capped at a ~2 cm drop -> squatting is the only way under.
        # LEG_JOINTS order: [yaw_l, roll_l, UPPER_L, LOWER_L, yaw_r, roll_r, UPPER_R, LOWER_R]
        self._squat_idx = (2, 3, 6, 7)
        self.leg_scale = np.full(self.n_leg, 0.25)
        self.leg_scale[list(self._squat_idx)] = 0.7
        self._squat_q = [self._q[LEG_JOINTS[i]] for i in self._squat_idx]
        self.p_table = p_table
        self.n_pillars = n_pillars

        n_extra = self.N_RAY_LOW + self.N_RAY_HIGH + 2
        self._frame_dim = self._frame_dim + n_extra          # 39 + 22 = 61
        from gymnasium import spaces as _sp
        self.observation_space = _sp.Box(
            -np.inf, np.inf, (self._frame_dim * self.k,), np.float32)

        # obstacle bookkeeping
        self._pillars = []
        for i in range(1, 5):
            try:
                self._pillars.append(int(self.model.body(f"obstacle_{i}").mocapid[0]))
            except Exception:
                pass
        self._table = int(self.model.body("table_top").mocapid[0])
        self._table_half = 0.02                                # slab half thickness
        # geom sets for collision counting: robot = dynamic bodies; obstacles = mocap
        # bodies with colliding geoms (the goal marker has contype 0 and never contacts)
        mocap_bodies = {b for b in range(self.model.nbody)
                        if self.model.body_mocapid[b] >= 0}
        self._obst_geoms = {g for g in range(self.model.ngeom)
                            if self.model.geom_bodyid[g] in mocap_bodies
                            and self.model.geom_contype[g]}
        self._robot_geoms = {g for g in range(self.model.ngeom)
                             if self.model.geom_bodyid[g] not in mocap_bodies
                             and self.model.geom_bodyid[g] != 0}
        # ray filter: ONLY group-2 geoms (obstacles / terrain). Group 2 is unused by the
        # robot (0 = collision, 1 = visual) and is RENDERED by default (groups 0-2), so
        # obstacles show up in videos / the live viewer without extra flags.
        self._raygroup = np.zeros(6, dtype=np.uint8)
        self._raygroup[2] = 1
        self._geomid_out = np.zeros(1, dtype=np.int32)

        # robot top height above the base origin (for duck clearance targets): measured
        # from the home pose so the table height can be set relative to the real robot.
        mujoco.mj_forward(self.model, self.data)
        top = 0.0
        for g in self._robot_geoms:
            top = max(top, float(self.data.geom_xpos[g][2]) +
                      float(np.max(self.model.geom_size[g])))
        self.top_off = max(0.05, top - self._spawn_z)          # ~body-top above base z
        # slab-bottom clearance range: low enough that STANDING collides, high enough
        # that a squat passes (base can drop ~0.15 by bending knees)
        self._clear_range = (self._spawn_z + self.top_off - 0.14,
                             self._spawn_z + self.top_off - 0.06)

        self.up_orig = 0.45        # up-ray cast origin: this far BELOW the base origin

        # extra reward weights
        self.w_col = 2.0           # per-step penalty while touching an obstacle
        self.w_prox = 0.3          # soft penalty when a low ray gets very close
        self.w_height = 3.0        # track the height target (nominal or duck target)
        self.duck_margin = 0.04    # keep robot top this far below the slab
        self._table_on = False
        self._table_z = 0.0        # slab BOTTOM height when placed

    # ---- perception ----------------------------------------------------------
    def _ray(self, pnt, vec, maxdist):
        d = mujoco.mj_ray(self.model, self.data, np.asarray(pnt, np.float64),
                          np.asarray(vec, np.float64), self._raygroup, 1, -1,
                          self._geomid_out)
        return maxdist if d < 0 else min(float(d), maxdist)

    def _rays(self):
        bx = float(self.data.qpos[self._base_qadr + 0])
        by = float(self.data.qpos[self._base_qadr + 1])
        bz = float(self.data.qpos[self._base_qadr + 2])
        yaw = self._base_yaw()
        low, high = [], []
        for i in range(self.N_RAY_LOW):
            th = yaw + 2.0 * math.pi * i / self.N_RAY_LOW
            low.append(self._ray([bx, by, max(bz - 0.35, 0.08)],
                                 [math.cos(th), math.sin(th), 0.0],
                                 self.RAY_MAX) / self.RAY_MAX)
        for i in range(self.N_RAY_HIGH):
            th = yaw + 2.0 * math.pi * i / self.N_RAY_HIGH
            high.append(self._ray([bx, by, bz + self.top_off],
                                  [math.cos(th), math.sin(th), 0.0],
                                  self.RAY_MAX) / self.RAY_MAX)
        # ceiling clearance: the base origin sits near the robot's TOP (top_off ~0.08)
        # and the duck slab's bottom can be BELOW it, so cast the up-ray from well
        # below the base and report the overhead height RELATIVE TO THE BASE (negative
        # = the base itself must sink under the slab bottom). Nothing overhead -> the
        # constant UP_MAX - up_orig.
        d_up = self._ray([bx, by, bz - self.up_orig], [0.0, 0.0, 1.0], self.UP_MAX)
        up_clear = d_up - self.up_orig
        # ground under the robot: down-ray onto group-2 terrain (stairs/hfield). On the
        # bare floor plane (not group 2) it misses -> ground level 0. Makes the height
        # features/shaping GROUND-RELATIVE, so stair climbing isn't punished as "too
        # high" and ducking works on elevated terrain too.
        dg = self._ray([bx, by, bz], [0.0, 0.0, -1.0], self.UP_MAX)
        ground = 0.0 if dg >= self.UP_MAX else bz - dg
        h = bz - ground                            # base height above local ground
        return low, high, up_clear, h, bz

    def _read_frame(self):
        base = super()._read_frame()               # 39-d Stage-2 prefix (unchanged)
        low, high, up, h, _bz = self._rays()
        extra = np.array(low + high + [up, h], dtype=np.float32)
        return np.concatenate([base, np.clip(np.nan_to_num(extra), -50.0, 50.0)])

    # ---- scene randomization ---------------------------------------------------
    def _place_obstacles(self):
        gx, gy = float(self.goal_xy[0]), float(self.goal_xy[1])
        path_yaw = math.atan2(gy, gx)
        r_goal = math.hypot(gx, gy)
        # everything buried unless placed below
        for m in self._pillars:
            self.data.mocap_pos[m] = [0.0, 0.0, -5.0]
        self.data.mocap_pos[self._table] = [0.0, 0.0, -5.0]
        self._table_on = bool(self._rng.uniform() < self.p_table)
        if self._table_on:
            # slab across the midpoint of the path, long axis perpendicular to it,
            # bottom low enough that standing collides but a squat passes
            clear = float(self._rng.uniform(*self._clear_range))
            self._table_z = clear
            mx, my = 0.5 * gx, 0.5 * gy
            self.data.mocap_pos[self._table] = [mx, my, clear + self._table_half]
            q = math.sin((path_yaw + math.pi / 2) / 2), math.cos((path_yaw + math.pi / 2) / 2)
            self.data.mocap_quat[self._table] = [q[1], 0.0, 0.0, q[0]]
        n_pil = int(self._rng.integers(self.n_pillars[0], self.n_pillars[1] + 1))
        placed = []
        for k, m in enumerate(self._pillars[:n_pil]):
            for _ in range(20):                     # rejection-sample a valid spot
                if k == 0 and not self._table_on:   # first pillar blocks the path
                    t = float(self._rng.uniform(0.4, 0.6))
                    off = float(self._rng.uniform(-0.3, 0.3))
                    px = t * gx - off * math.sin(path_yaw)
                    py = t * gy + off * math.cos(path_yaw)
                else:                               # the rest scatter around the course
                    ang = float(self._rng.uniform(-math.pi, math.pi))
                    rr = float(self._rng.uniform(0.6, max(r_goal, 0.9)))
                    px, py = rr * math.cos(ang), rr * math.sin(ang)
                ok = (math.hypot(px, py) > 0.5 and
                      math.hypot(px - gx, py - gy) > 0.5 and
                      all(math.hypot(px - qx, py - qy) > 0.45 for qx, qy in placed))
                if self._table_on:                  # keep pillars off the table zone
                    mx, my = 0.5 * gx, 0.5 * gy
                    ok = ok and math.hypot(px - mx, py - my) > 1.0
                if ok:
                    self.data.mocap_pos[m] = [px, py, 0.40]
                    placed.append((px, py))
                    break

    def reset(self, seed=None, options=None):
        obs, info = super().reset(seed=seed, options=options)
        self._place_obstacles()
        mujoco.mj_forward(self.model, self.data)
        # rebuild the history with the obstacles actually in place
        f = self._read_frame()
        self._hist.clear()
        for _ in range(self.k):
            self._hist.append(f)
        return self._obs(), info

    # ---- step: nav reward + collision/proximity/duck terms ---------------------
    def _collided(self):
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if ((g1 in self._obst_geoms and g2 in self._robot_geoms) or
                    (g2 in self._obst_geoms and g1 in self._robot_geoms)):
                return True
        return False

    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)
        low, _high, up_clear, h, bz = self._rays()
        # collision + proximity
        hit = self._collided()
        if hit:
            reward -= self.w_col
        min_low = min(low) * self.RAY_MAX
        if min_low < 0.35:
            reward -= self.w_prox * (0.35 - min_low) / 0.35
        # height target (GROUND-RELATIVE): nominal stance height normally; under the
        # slab, drop so the robot's top clears it. Also refund most of the posture
        # penalty while ducking (bent legs are the POINT there, not a fault).
        ducking = up_clear < (self.top_off + 0.15)
        h_tgt = self._spawn_z
        if ducking:
            h_tgt = min(self._spawn_z, h + up_clear - self.top_off - self.duck_margin)
            h_tgt = max(h_tgt, 0.30)
            # refund the posture penalty ONLY for the squat (pitch) joints -- bending
            # the knees under the table is the point. Hip yaw/roll deviation stays
            # fully penalized, so SQUATTING (not leg-splaying) is the cheap way under.
            squat_dev = float(sum(self.data.qpos[qi] ** 2 for qi in self._squat_q))
            reward += 0.85 * self.w_pose * squat_dev
        reward -= self.w_height * (h - h_tgt) ** 2
        # collapsed (fell through the squat) counts as a fall
        if h < 0.25:
            terminated = True
            reward -= 5.0
        info.update({"collision": hit, "up_clear": up_clear, "base_z": bz,
                     "height": h, "ducking": ducking})
        return obs, float(reward), terminated, truncated, info


# ============================================================================ #
#  Stage 4 — stairs + rough terrain (on top of everything from Stages 2-3)     #
# ============================================================================ #
class TerrainNavEnv(ObstacleNavEnv):
    """Stage-4: same robot/skills, but the scene (mybot_terrain.xml, generated by
    terrain.py) adds a fixed STAIRCASE and a ROUGH heightfield patch. Each episode is
    one of three modes: 'flat' (a Stage-3 obstacle course episode — earlier skills keep
    being rehearsed, no catastrophic forgetting), 'stairs' (goal on/atop the steps, a
    small climb bonus on ground height gained), or 'rough' (goal inside the bump field,
    which is re-randomized every reset). Obs/action identical to Stage 3 (the rays and
    the down-ray see the terrain because it is group 3), so Stage-3 checkpoints
    warm-start directly."""

    def __init__(self, render=False, control_hz=50.0, max_steps=1000, seed=None,
                 model_path=None):
        from terrain import (ensure_terrain_model, STAIR_X0, STAIR_RUN, STAIR_N,
                             ROUGH_C, ROUGH_HALF, HF_RES)
        super().__init__(render=render, control_hz=control_hz, max_steps=max_steps,
                         seed=seed, p_table=0.5, n_pillars=(1, 3),
                         model_path=model_path or ensure_terrain_model())
        self._stair_x0, self._stair_run, self._stair_n = STAIR_X0, STAIR_RUN, STAIR_N
        self._rough_c, self._rough_half, self._hf_res = ROUGH_C, ROUGH_HALF, HF_RES
        self._mode = "flat"
        self.w_climb = 1.0                 # bonus per step on ground height gained
        try:
            hf = self.model.hfield("rough")
            self._hf_adr, self._hf_n = int(hf.adr[0]), int(hf.nrow[0]) * int(hf.ncol[0])
        except Exception:
            self._hf_adr = -1

    # ---- episode modes ---------------------------------------------------------
    def _sample_goal(self):
        r = float(self._rng.uniform())
        self._mode = "flat" if r < 0.30 else ("stairs" if r < 0.65 else "rough")
        if self._mode == "flat":
            super()._sample_goal()
        elif self._mode == "stairs":
            total = self._stair_run * self._stair_n
            gx = self._stair_x0 + float(self._rng.uniform(0.5 * total, total + 0.6))
            gy = float(self._rng.uniform(-0.6, 0.6))
            self.goal_xy = np.array([gx, gy], np.float32)
        else:                                                    # rough patch
            cx, cy = self._rough_c
            m = self._rough_half - 0.4
            self.goal_xy = np.array([cx + float(self._rng.uniform(-m, m)),
                                     cy + float(self._rng.uniform(-m, m))], np.float32)

    def _place_obstacles(self):
        if self._mode == "flat":
            super()._place_obstacles()                           # full Stage-3 course
        else:                                                    # clear terrain runs
            for m in self._pillars:
                self.data.mocap_pos[m] = [0.0, 0.0, -5.0]
            self.data.mocap_pos[self._table] = [0.0, 0.0, -5.0]
            self._table_on = False

    def _randomize_rough(self):
        if self._hf_adr < 0:
            return
        res = self._hf_res
        coarse = self._rng.uniform(0.0, 1.0, (8, 8))
        up = np.kron(coarse, np.ones((res // 8, res // 8)))       # upsample
        for _ in range(3):                                        # cheap box blur
            up = 0.2 * (up + np.roll(up, 1, 0) + np.roll(up, -1, 0)
                        + np.roll(up, 1, 1) + np.roll(up, -1, 1))
        up = (up - up.min()) / max(up.max() - up.min(), 1e-6)
        self.model.hfield_data[self._hf_adr:self._hf_adr + self._hf_n] = \
            up.ravel().astype(np.float32)

    def reset(self, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._randomize_rough()                    # new bumps every episode
        obs, info = super().reset(seed=None, options=options)
        if self._mode == "stairs" and self._goal_mocap >= 0:      # lift marker onto steps
            self.data.mocap_pos[self._goal_mocap][2] = 0.35
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)
        if self._mode == "stairs":
            ground = float(info["base_z"]) - float(info["height"])
            reward += self.w_climb * max(0.0, min(ground, 0.30))  # reward height gained
        info["mode"] = self._mode
        return obs, float(reward), terminated, truncated, info
