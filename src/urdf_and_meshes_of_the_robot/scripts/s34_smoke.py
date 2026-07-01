#!/usr/bin/env python3
"""Smoke test for the Stage-3 (obstacles+duck) and Stage-4 (terrain) envs + the
weight-surgery warm-start. Throwaway validation script."""
import numpy as np
import torch

from nav_env_mujoco import ObstacleNavEnv, TerrainNavEnv

print("== Stage 3: ObstacleNavEnv ==")
env = ObstacleNavEnv(seed=0)
print(f"frame_dim={env._frame_dim} obs={env.observation_space.shape} "
      f"act={env.action_space.shape} top_off={env.top_off:.3f} "
      f"clear_range=({env._clear_range[0]:.3f},{env._clear_range[1]:.3f}) "
      f"spawn_z={env._spawn_z:.3f}")
assert env._frame_dim == 61 and env.observation_space.shape == (183,)
obs, _ = env.reset(seed=3)
assert obs.shape == (183,) and np.isfinite(obs).all()
# ray sanity: place a pillar 1 m straight ahead (+x, yaw=0) -> forward low ray sees it
env.data.mocap_pos[env._pillars[0]] = [1.0, 0.0, 0.40]
import mujoco
mujoco.mj_forward(env.model, env.data)
low, high, up_clear, h, bz = env._rays()
print(f"fwd low ray={low[0]*env.RAY_MAX:.2f}m (expect ~0.88)  up_clear={up_clear:.2f} "
      f"(clear-sky const {env.UP_MAX - env.up_orig:.2f})  h={h:.2f} bz={bz:.2f}")
assert 0.6 < low[0] * env.RAY_MAX < 1.05, "forward ray should hit the pillar"
assert abs(up_clear - (env.UP_MAX - env.up_orig)) < 1e-6, "clear sky value"
# table above -> up ray sees it; clearance goes small/negative (slab below base origin)
env.data.mocap_pos[env._table] = [env.data.qpos[0], env.data.qpos[1], env._clear_range[0] + 0.02]
mujoco.mj_forward(env.model, env.data)
_, _, up2, h2, _ = env._rays()
print(f"under table: up_clear={up2:.3f} (slab bottom {env._clear_range[0]:.3f}, base {bz:.2f})")
assert up2 < env.top_off + 0.15, "should register as duck zone"
assert abs(up2 - (env._clear_range[0] - bz)) < 0.02, "clearance = slab bottom - base z"
for t in range(30):
    obs, r, term, trunc, info = env.step(env.action_space.sample() * 0.2)
    assert np.isfinite(obs).all()
    if term or trunc:
        break
print(f"stepped ok; info keys={sorted(info.keys())}")
env.close()

print("== Stage 4: TerrainNavEnv ==")
env = TerrainNavEnv(seed=0)
assert env.observation_space.shape == (183,)
modes = {}
for ep in range(12):
    obs, _ = env.reset(seed=100 + ep)
    modes[env._mode] = modes.get(env._mode, 0) + 1
    assert np.isfinite(obs).all()
    obs, r, term, trunc, info = env.step(np.zeros(10, np.float32))
    assert np.isfinite(obs).all()
print("episode modes over 12 resets:", modes)
assert len(modes) >= 2, "should see multiple modes"
# ground-relative height: teleport base over the stairs top platform
from terrain import STAIR_X0, STAIR_RUN, STAIR_N, STAIR_RISE
top_x = STAIR_X0 + STAIR_RUN * STAIR_N + 0.5
env.data.qpos[0], env.data.qpos[1] = top_x, 0.0
env.data.qpos[2] = STAIR_RISE * STAIR_N + 0.52
mujoco.mj_forward(env.model, env.data)
low, high, up, h, bz = env._rays()
print(f"on stairs top: bz={bz:.3f} ground-relative h={h:.3f} (expect ~0.52)")
assert abs(h - 0.52) < 0.08, "down-ray should measure height above the platform"
env.close()

print("== weight surgery: Stage-2 ckpt (117) -> Stage-3 agent (183) ==")
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ppo_balance import ActorCritic, RunningMeanStd
from ppo_parallel import _surgery_load
ck = torch.load("/mnt/c/Users/mtasfaw/Documents/personal/mybotV1-DRL-BASED/"
                "training_results/nav2b_goal/model_best.pth",
                map_location="cpu", weights_only=False)
agent = ActorCritic(183, 10)
rms = RunningMeanStd((183,))
_surgery_load(agent, rms, ck, 183, 10, 117, 10, 61, 3, print)
w_new = agent.state_dict()["actor_mean.0.weight"]
w_old = ck["model"]["actor_mean.0.weight"]
# old frame col 5 of frame 1 -> new position 1*61+5 ; new ray col (1*61+45) must be 0
assert torch.allclose(w_new[:, 1 * 61 + 5], w_old[:, 1 * 39 + 5])
assert float(w_new[:, 1 * 61 + 45].abs().sum()) == 0.0
assert np.allclose(rms.mean[2 * 61 + 7], np.asarray(ck["obs_rms"]["mean"])[2 * 39 + 7])
print("surgery mapping verified (old cols copied, new ray cols zero, rms mapped)")
print("S34 SMOKE OK")
