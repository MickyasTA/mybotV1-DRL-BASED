#!/usr/bin/env python3
"""
Watch a trained policy balance LIVE in a MuJoCo window (interactive — drag to orbit).

  DISPLAY=:0 MUJOCO_GL=egl python mujoco_play.py --ckpt <run>/model_best.pth

Needs a display (WSLg's :0 on WSL). For headless machines use mujoco_record.py -> mp4.
"""
import os
import sys
import time
import argparse
import numpy as np
import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)


def play(ckpt, loops=1000):
    from ppo_balance import ActorCritic, RunningMeanStd
    from balance_env_mujoco import MujocoBalanceEnv
    env = MujocoBalanceEnv(render=True)          # live viewer
    obs_dim = env.observation_space.shape[0]
    act_dim = env.action_space.shape[0]
    agent = ActorCritic(obs_dim, act_dim)
    obs_rms = RunningMeanStd((obs_dim,))
    ck = torch.load(ckpt, map_location="cpu")
    agent.load_state_dict(ck["model"])
    obs_rms.load_state_dict(ck["obs_rms"])
    agent.eval()
    dt = env.sim_dt * env.n_sub                  # control period for real-time pacing
    for ep in range(loops):
        obs, _ = env.reset()
        done, steps = False, 0
        while not done:
            n = obs_rms.normalize(obs).astype(np.float32)
            with torch.no_grad():
                a = agent.actor_mean(torch.as_tensor(n)).numpy()
            obs, r, term, trunc, info = env.step(a)   # env.render() runs inside (human mode)
            time.sleep(dt)
            steps += 1
            done = term or trunc
        print(f"episode {ep}: balanced {steps} steps ({steps*dt:.1f}s)"
              + ("  [fell]" if term else "  [full]"))
    env.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", required=True)
    p.add_argument("--loops", type=int, default=1000)
    a = p.parse_args()
    play(a.ckpt, a.loops)
