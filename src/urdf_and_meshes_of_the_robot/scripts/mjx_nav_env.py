#!/usr/bin/env python3
"""
Functional MJX goal-navigation env (GPU twin of nav_env_mujoco.py). Reuses the model +
kinematics from mjx_env and adds: a base-frame GOAL vector in the obs, a 10-D action
(2 wheels + 8 leg position targets), and the goal/progress/station-keep reward PLUS the
posture penalty that stops the leg-splaying. Same obs/action layout as the CPU nav env
(obs 117, act 10), so policies are comparable and portable between CPU and GPU.
"""
import math
import numpy as np
import jax
import jax.numpy as jp
import mujoco
from mujoco import mjx
from flax import struct

import mjx_env as B      # shared model (MX), indices, kinematics

MX = B.MX
_MJ = B._MJ
SIM_DT, N_SUB, MAX_STEPS, K = B.SIM_DT, B.N_SUB, B.MAX_STEPS, B.K
BASE_Q, BASE_V = B.BASE_Q, B.BASE_V
WHEEL_V, WHEEL_ACT = B.WHEEL_V, B.WHEEL_ACT
QPOS0, SPAWN_Z = B.QPOS0, B.SPAWN_Z
LEG_JOINTS = B.LEG_JOINTS

LEG_Q = tuple(int(_MJ.joint(j).qposadr[0]) for j in LEG_JOINTS)
LEG_V = tuple(int(_MJ.joint(j).dofadr[0]) for j in LEG_JOINTS)
LEG_ACT = tuple(B._act_for(j) for j in LEG_JOINTS)
LEG_LO = jp.array([_MJ.actuator_ctrlrange[a, 0] for a in LEG_ACT])
LEG_HI = jp.array([_MJ.actuator_ctrlrange[a, 1] for a in LEG_ACT])
_LEG_ACT_ARR = jp.array(LEG_ACT)
_WHEEL_ACT_ARR = jp.array(WHEEL_ACT)

ACT_DIM = 2 + len(LEG_JOINTS)                 # 10
FRAME_DIM = 9 + len(LEG_JOINTS) * 2 + 4 + ACT_DIM   # 39
OBS_DIM = FRAME_DIM * K                        # 117
MAX_WHEEL_TORQUE = 20.0
LEG_SCALE = 0.4

# reward weights (mirror nav_env_mujoco.py)
W_TILT, W_YAW, W_POSE, W_ACT, W_ARATE = 2.0, 0.15, 0.6, 0.005, 0.02
W_PROG, W_VEL, W_FACE = 10.0, 0.3, 0.3
W_ARRIVE, W_STOP, W_STALL = 5.0, 1.0, 0.5
FALL_TILT, GOAL_TOL = 0.6, 0.25
GOAL_MIN_R, GOAL_MAX_R = 0.8, 2.0


@struct.dataclass
class State:
    data: mjx.Data
    hist: jp.ndarray
    last_action: jp.ndarray
    obs: jp.ndarray
    reward: jp.ndarray
    done: jp.ndarray
    step: jp.ndarray
    key: jp.ndarray
    goal: jp.ndarray          # (2,) world xy
    prev_dist: jp.ndarray     # scalar


def _yaw(quat):
    w, x, y, z = quat
    return jp.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _goal_feats(data, goal):
    bx, by = data.qpos[BASE_Q + 0], data.qpos[BASE_Q + 1]
    dx, dy = goal[0] - bx, goal[1] - by
    dist = jp.sqrt(dx * dx + dy * dy)
    yaw = _yaw(data.qpos[BASE_Q + 3: BASE_Q + 7])
    c, s = jp.cos(-yaw), jp.sin(-yaw)
    bxr = c * dx - s * dy
    byr = s * dx + c * dy
    heading_err = jp.arctan2(byr, bxr)
    n = jp.maximum(dist, 1e-6)
    return jp.array([bxr / n, byr / n]), heading_err, dist, dx, dy


def _frame(data, last_action, goal):
    quat = data.qpos[BASE_Q + 3: BASE_Q + 7]
    roll, pitch, _ = B._rpy_tilt(quat)
    wx, wy, wz = data.qvel[BASE_V + 3], data.qvel[BASE_V + 4], data.qvel[BASE_V + 5]
    bvx, bvy = data.qvel[BASE_V + 0], data.qvel[BASE_V + 1]
    wl, wr = data.qvel[WHEEL_V[0]], data.qvel[WHEEL_V[1]]
    leg_pos = jp.array([data.qpos[i] for i in LEG_Q])
    leg_vel = jp.array([data.qvel[i] for i in LEG_V])
    gdir, gher, gdist, _, _ = _goal_feats(data, goal)
    base = jp.array([roll, pitch, wx, wy, wz, wl, wr, bvx, bvy])
    frame = jp.concatenate([base, leg_pos, leg_vel,
                            jp.array([gdir[0], gdir[1], gher, gdist]), last_action])
    return jp.clip(jp.nan_to_num(frame), -50.0, 50.0)


def reset(key):
    key, kg, kp = jax.random.split(key, 3)
    ang = jax.random.uniform(kg, (), minval=-math.pi, maxval=math.pi)
    r = jax.random.uniform(kg, (), minval=GOAL_MIN_R, maxval=GOAL_MAX_R)
    goal = jp.array([r * jp.cos(ang), r * jp.sin(ang)])
    dr = jax.random.uniform(kp, (2,), minval=-0.05, maxval=0.05)
    cr, sr = jp.cos(dr[0] / 2), jp.sin(dr[0] / 2)
    cp, sp = jp.cos(dr[1] / 2), jp.sin(dr[1] / 2)
    quat = jp.array([cr * cp, sr * cp, cr * sp, -sr * sp])
    qpos = QPOS0.at[BASE_Q + 3: BASE_Q + 7].set(quat).at[BASE_Q + 2].set(SPAWN_Z)
    data = mjx.make_data(MX).replace(qpos=qpos, qvel=jp.zeros(_MJ.nv), ctrl=jp.zeros(_MJ.nu))
    data = mjx.forward(MX, data)
    la = jp.zeros(ACT_DIM)
    frame = _frame(data, la, goal)
    hist = jp.tile(frame, (K, 1))
    return State(data=data, hist=hist, last_action=la, obs=hist.reshape(-1),
                 reward=jp.float32(0.0), done=jp.float32(0.0), step=jp.int32(0),
                 key=key, goal=goal, prev_dist=jp.sqrt(goal[0] ** 2 + goal[1] ** 2))


def step(state, action):
    a = jp.clip(action, -1.0, 1.0)
    prev_a = state.last_action
    leg_t = jp.clip(LEG_SCALE * a[2:], LEG_LO, LEG_HI)
    ctrl = (jp.zeros(_MJ.nu)
            .at[_WHEEL_ACT_ARR].set(a[:2] * MAX_WHEEL_TORQUE)
            .at[_LEG_ACT_ARR].set(leg_t))
    data = state.data.replace(ctrl=ctrl)
    data = jax.lax.fori_loop(0, N_SUB, lambda _, d: mjx.step(MX, d), data)

    frame = _frame(data, a, state.goal)
    hist = jp.concatenate([state.hist[1:], frame[None]], axis=0)
    obs = hist.reshape(-1)

    quat = data.qpos[BASE_Q + 3: BASE_Q + 7]
    _, _, tilt = B._rpy_tilt(quat)
    wz = data.qvel[BASE_V + 5]
    bvx, bvy = data.qvel[BASE_V + 0], data.qvel[BASE_V + 1]
    speed = jp.sqrt(bvx * bvx + bvy * bvy)
    _, heading_err, dist, dx, dy = _goal_feats(data, state.goal)
    progress = state.prev_dist - dist
    dn = jp.maximum(dist, 1e-6)
    v_to_goal = (dx * bvx + dy * bvy) / dn
    leg_dev = jp.sum(jp.array([data.qpos[i] ** 2 for i in LEG_Q]))
    at_goal = dist < GOAL_TOL

    reward = (1.0
              - W_TILT * tilt * tilt - W_YAW * wz * wz - W_POSE * leg_dev
              - W_ACT * jp.sum(a * a) - W_ARATE * jp.sum((a - prev_a) ** 2)
              + W_PROG * progress + W_VEL * jp.clip(v_to_goal, -1.5, 1.5)
              + W_FACE * jp.cos(heading_err))
    reward = reward + jp.where(at_goal, W_ARRIVE - W_STOP * speed * speed,
                               jp.where((speed < 0.1) & (dist > 0.5), -W_STALL, 0.0))
    nstep = state.step + 1
    fell = tilt > FALL_TILT
    reward = reward - 5.0 * fell
    done = jp.float32(jp.logical_or(fell, nstep >= MAX_STEPS))

    live = State(data=data, hist=hist, last_action=a, obs=obs,
                 reward=reward.astype(jp.float32), done=done, step=nstep,
                 key=state.key, goal=state.goal, prev_dist=dist)
    key, sub = jax.random.split(state.key)
    rs = reset(sub)
    d = done > 0.5
    return live.replace(
        data=jax.tree_util.tree_map(lambda l, r: jp.where(d, r, l), live.data, rs.data),
        hist=jp.where(d, rs.hist, live.hist),
        obs=jp.where(d, rs.obs, live.obs),
        last_action=jp.where(d, rs.last_action, live.last_action),
        step=jp.where(d, rs.step, live.step),
        goal=jp.where(d, rs.goal, live.goal),
        prev_dist=jp.where(d, rs.prev_dist, live.prev_dist),
        key=key)


if __name__ == "__main__":
    print(f"OBS_DIM={OBS_DIM} ACT_DIM={ACT_DIM} FRAME_DIM={FRAME_DIM} LEG_ACT={LEG_ACT}")
    N = 256
    st = jax.jit(jax.vmap(reset))(jax.random.split(jax.random.PRNGKey(0), N))
    print("reset obs", st.obs.shape, "finite", bool(jp.isfinite(st.obs).all()),
          "goal0", np.round(np.asarray(st.goal[0]), 3))
    vstep = jax.jit(jax.vmap(step))
    st = vstep(st, jp.zeros((N, ACT_DIM))); st.obs.block_until_ready()
    print("step OK; reward~%.3f done~%.3f" % (float(st.reward.mean()), float(st.done.mean())))
