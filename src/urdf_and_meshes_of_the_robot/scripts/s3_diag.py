#!/usr/bin/env python3
"""Diagnose why the surgically-transferred Stage-2 policy dies in ObstacleNavEnv."""
import numpy as np
import torch

from ppo_balance import ActorCritic, RunningMeanStd
from ppo_parallel import _surgery_load
from nav_env_mujoco import ObstacleNavEnv

CK = ("/mnt/c/Users/mtasfaw/Documents/personal/mybotV1-DRL-BASED/"
      "training_results/nav3b_goal/model_best.pth")
ck = torch.load(CK, map_location="cpu", weights_only=False)

agent = ActorCritic(183, 10)
rms = RunningMeanStd((183,))
scale = np.ones(10); scale[2:] = 0.25 / 0.7
_surgery_load(agent, rms, ck, 183, 10, 117, 10, 61, 3, lambda s: None,
              act_scale=scale)
agent.eval()


def run(tag, **envkw):
    env = ObstacleNavEnv(seed=0, **envkw)
    for ep in range(3):
        obs, _ = env.reset(seed=10 + ep)
        info, t = {}, 0
        for t in range(1000):
            n = rms.normalize(obs).astype(np.float32)
            with torch.no_grad():
                a = agent.actor_mean(torch.as_tensor(n)).numpy()
            obs, r, term, trunc, info = env.step(a)
            if term or trunc:
                break
        cause = "cap"
        if term:
            cause = "TILT" if info["tilt"] > env.fall_tilt else "COLLAPSE(h<0.25)"
        print(f"  [{tag}] ep{ep}: steps={t+1} cause={cause} tilt={info['tilt']:.2f} "
              f"h={info['height']:.2f} duck={info['ducking']} col={info['collision']} "
              f"dist={info['dist']:.2f}")
    env.close()


print("== obstacles OFF (pure transfer sanity) ==")
run("no-obst", p_table=0.0, n_pillars=(0, 0))
print("== obstacles ON (the real Stage-3 setting) ==")
run("obst")
