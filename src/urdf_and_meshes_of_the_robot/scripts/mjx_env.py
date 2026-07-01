#!/usr/bin/env python3
"""
Functional, batched MJX (MuJoCo-on-GPU) env for the wheel-legged robot — the JAX twin
of balance_env_mujoco.py. Pure-JAX reset/step so thousands of envs run in parallel on
the GPU via jax.vmap; the from-scratch JAX PPO (mjx_ppo.py) scans over these.

Stage 1 (balance) matches the PyTorch env's obs/reward exactly so GPU and CPU results
are comparable:
  * obs: 3-frame history of [roll,pitch,wx,wy,wz,wl,wr,bvx,bvy] + hip_pos(6)+hip_vel(6)
         + last_action(2)  -> 23*3 = 69
  * action: 2 wheel torques in [-1,1] (scaled to +-20 Nm); 8 legs held at 0 stance
  * reward: 1 - 2*tilt^2 - 0.05*drift^2 - 0.15*yaw_rate^2 - 0.01*||a||^2 ; term tilt>0.6

MJX-compat: MJX has no cylinder-mesh/mesh-mesh collision, so we disable collision on the
mesh geoms (they are in the air on flat ground) -> only wheel-cylinder vs floor-plane.
"""
import os
import numpy as np
import jax
import jax.numpy as jp
import mujoco
from mujoco import mjx
from flax import struct

_HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.abspath(os.path.join(_HERE, "..", "mujoco", "mybot.xml"))

WHEEL_JOINTS = ["Left_joint", "Right_joint"]
HIP_JOINTS = ["hip_yaw_left", "hip_roll_left", "Upper_left_joint",
              "hip_yaw_right", "hip_roll_right", "Upper_right_joint"]
LEG_JOINTS = ["hip_yaw_left", "hip_roll_left", "Upper_left_joint", "Lower_left_joint",
              "hip_yaw_right", "hip_roll_right", "Upper_right_joint", "Lower_right_joint"]


def _load_mjx_model():
    """CPU MjModel with mesh collisions disabled -> MJX-compatible -> put on device."""
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    MESH = mujoco.mjtGeom.mjGEOM_MESH
    for g in range(m.ngeom):
        if m.geom_type[g] == MESH:
            m.geom_contype[g] = 0
            m.geom_conaffinity[g] = 0
    m.opt.cone = 0            # pyramidal friction cone: ~2x faster in MJX than elliptic
    return m, mjx.put_model(m)


# ---- static model + index bookkeeping (plain Python ints, baked into the jitted fns) --
_MJ, MX = _load_mjx_model()
SIM_DT = float(_MJ.opt.timestep)
N_SUB = 10                                   # 50 Hz control over 2 ms physics
MAX_STEPS = 1000
K = 3                                        # history frames
FRAME_DIM = 9 + len(HIP_JOINTS) * 2 + 2      # 23
OBS_DIM = FRAME_DIM * K                       # 69
ACT_DIM = len(WHEEL_JOINTS)                   # 2
MAX_WHEEL_TORQUE = 20.0

_qadr = lambda n: int(_MJ.joint(n).qposadr[0])
_vadr = lambda n: int(_MJ.joint(n).dofadr[0])


def _act_for(joint):
    jid = int(_MJ.joint(joint).id)
    for a in range(_MJ.nu):
        if int(_MJ.actuator_trnid[a, 0]) == jid:
            return a
    raise ValueError(joint)


BASE_Q, BASE_V = 0, 0                          # free joint at the front
WHEEL_V = tuple(_vadr(j) for j in WHEEL_JOINTS)
HIP_Q = tuple(_qadr(j) for j in HIP_JOINTS)
HIP_V = tuple(_vadr(j) for j in HIP_JOINTS)
WHEEL_ACT = tuple(_act_for(j) for j in WHEEL_JOINTS)
QPOS0 = jp.array(_MJ.qpos0)
SPAWN_Z = float(_MJ.qpos0[BASE_Q + 2])

W_TILT, W_DRIFT, W_YAW, W_ACT, FALL_TILT = 2.0, 0.05, 0.15, 0.01, 0.6


@struct.dataclass
class State:
    data: mjx.Data
    hist: jp.ndarray          # (K, FRAME_DIM)
    last_action: jp.ndarray   # (ACT_DIM,)
    obs: jp.ndarray           # (OBS_DIM,)
    reward: jp.ndarray        # scalar
    done: jp.ndarray          # scalar
    step: jp.ndarray          # scalar int
    key: jp.ndarray


# ---- kinematics from the base quaternion --------------------------------------------
def _rpy_tilt(quat):
    w, x, y, z = quat
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    r20 = 2.0 * (x * z - w * y)
    roll = jp.arctan2(r21, r22)
    pitch = jp.arctan2(-r20, jp.sqrt(r21 * r21 + r22 * r22))
    tilt = jp.arccos(jp.clip(r22, -1.0, 1.0))
    return roll, pitch, tilt


def _frame(data, last_action):
    quat = data.qpos[BASE_Q + 3: BASE_Q + 7]
    roll, pitch, _ = _rpy_tilt(quat)
    wx, wy, wz = data.qvel[BASE_V + 3], data.qvel[BASE_V + 4], data.qvel[BASE_V + 5]
    bvx, bvy = data.qvel[BASE_V + 0], data.qvel[BASE_V + 1]
    wl, wr = data.qvel[WHEEL_V[0]], data.qvel[WHEEL_V[1]]
    hip_pos = jp.array([data.qpos[i] for i in HIP_Q])
    hip_vel = jp.array([data.qvel[i] for i in HIP_V])
    base = jp.array([roll, pitch, wx, wy, wz, wl, wr, bvx, bvy])
    frame = jp.concatenate([base, hip_pos, hip_vel, last_action])
    return jp.clip(jp.nan_to_num(frame), -50.0, 50.0)


def _obs_from_hist(hist):
    return hist.reshape(-1)


def reset(key):
    key, sub = jax.random.split(key)
    dr = jax.random.uniform(sub, (2,), minval=-0.05, maxval=0.05)   # small roll/pitch
    # quaternion for (roll=dr0, pitch=dr1, yaw=0)
    cr, sr = jp.cos(dr[0] / 2), jp.sin(dr[0] / 2)
    cp, sp = jp.cos(dr[1] / 2), jp.sin(dr[1] / 2)
    quat = jp.array([cr * cp, sr * cp, cr * sp, -sr * sp])
    qpos = QPOS0.at[BASE_Q + 3: BASE_Q + 7].set(quat).at[BASE_Q + 2].set(SPAWN_Z)
    data = mjx.make_data(MX).replace(qpos=qpos, qvel=jp.zeros(_MJ.nv), ctrl=jp.zeros(_MJ.nu))
    data = mjx.forward(MX, data)
    la = jp.zeros(ACT_DIM)
    frame = _frame(data, la)
    hist = jp.tile(frame, (K, 1))
    return State(data=data, hist=hist, last_action=la, obs=_obs_from_hist(hist),
                 reward=jp.float32(0.0), done=jp.float32(0.0),
                 step=jp.int32(0), key=key)


def _physics(data, ctrl):
    data = data.replace(ctrl=ctrl)
    return jax.lax.fori_loop(0, N_SUB, lambda _, d: mjx.step(MX, d), data)


def step(state, action):
    a = jp.clip(action, -1.0, 1.0)
    ctrl = jp.zeros(_MJ.nu).at[jp.array(WHEEL_ACT)].set(a * MAX_WHEEL_TORQUE)  # legs held at 0
    data = _physics(state.data, ctrl)

    frame = _frame(data, a)
    hist = jp.concatenate([state.hist[1:], frame[None]], axis=0)
    obs = _obs_from_hist(hist)

    quat = data.qpos[BASE_Q + 3: BASE_Q + 7]
    _, _, tilt = _rpy_tilt(quat)
    wz = data.qvel[BASE_V + 5]
    drift2 = data.qpos[BASE_Q + 0] ** 2 + data.qpos[BASE_Q + 1] ** 2
    reward = (1.0 - W_TILT * tilt * tilt - W_DRIFT * drift2
              - W_YAW * wz * wz - W_ACT * jp.sum(a * a))
    nstep = state.step + 1
    fell = tilt > FALL_TILT
    reward = reward - 5.0 * fell
    done = jp.float32(jp.logical_or(fell, nstep >= MAX_STEPS))

    live = State(data=data, hist=hist, last_action=a, obs=obs,
                 reward=reward.astype(jp.float32), done=done, step=nstep, key=state.key)
    # autoreset where done: swap in a fresh episode's carry, keep this step's reward/done
    key, sub = jax.random.split(state.key)
    rs = reset(sub)
    d = done > 0.5
    return live.replace(
        data=jax.tree_util.tree_map(lambda l, r: jp.where(d, r, l), live.data, rs.data),
        hist=jp.where(d, rs.hist, live.hist),
        obs=jp.where(d, rs.obs, live.obs),
        last_action=jp.where(d, rs.last_action, live.last_action),
        step=jp.where(d, rs.step, live.step),
        key=key,
    )


if __name__ == "__main__":
    print(f"OBS_DIM={OBS_DIM} ACT_DIM={ACT_DIM} FRAME_DIM={FRAME_DIM} nu={_MJ.nu} "
          f"WHEEL_ACT={WHEEL_ACT} backend={jax.default_backend()}")
    N = 512
    keys = jax.random.split(jax.random.PRNGKey(0), N)
    st = jax.jit(jax.vmap(reset))(keys)
    print("reset: obs", st.obs.shape, "finite", bool(jp.isfinite(st.obs).all()))
    vstep = jax.jit(jax.vmap(step))
    import time
    act = jp.zeros((N, ACT_DIM))
    st = vstep(st, act); st.obs.block_until_ready()          # compile
    t0 = time.time()
    T = 200
    for _ in range(T):
        st = vstep(st, act)
    st.obs.block_until_ready()
    dt = time.time() - t0
    print(f"rollout {N} envs x {T} ctrl-steps ({T*N_SUB} phys) in {dt:.2f}s = "
          f"{N*T/dt:,.0f} ctrl-steps/s  reward~{float(st.reward.mean()):.3f} "
          f"done_frac~{float(st.done.mean()):.3f}")
