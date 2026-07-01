#!/usr/bin/env python3
"""
Record an mp4 of a trained policy balancing in MuJoCo (offscreen, no display needed).

  MUJOCO_GL=egl python mujoco_record.py --ckpt <model.pth> --out clip.mp4

Loads the from-scratch PPO checkpoint (ActorCritic + obs normalizer), runs one
deterministic episode in MujocoBalanceEnv, renders each control step offscreen, and
writes an mp4. Falls back osmesa->egl automatically for headless GL.
"""
import os
# headless GL backend must be chosen BEFORE mujoco is imported (via the env import)
os.environ.setdefault("MUJOCO_GL", "egl")
import sys
import argparse
import numpy as np
import torch
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)


def _make(env_kwargs):
    from balance_env_mujoco import MujocoBalanceEnv
    return MujocoBalanceEnv(**env_kwargs)


def record(ckpt_path, out_path, max_steps=1000, fps=50, seed=0):
    from ppo_balance import ActorCritic, RunningMeanStd
    try:
        env = _make(dict(max_steps=max_steps))
    except Exception as e:                       # GL backend fallback
        os.environ["MUJOCO_GL"] = "osmesa"
        env = _make(dict(max_steps=max_steps))
    obs_dim = env.observation_space.shape[0]
    act_dim = env.action_space.shape[0]
    agent = ActorCritic(obs_dim, act_dim)
    obs_rms = RunningMeanStd((obs_dim,))
    ckpt = torch.load(ckpt_path, map_location="cpu")
    agent.load_state_dict(ckpt["model"])
    obs_rms.load_state_dict(ckpt["obs_rms"])
    agent.eval()

    obs, _ = env.reset(seed=seed)
    frames, ep_len = [], 0
    for _ in range(max_steps):
        n = obs_rms.normalize(obs).astype(np.float32)
        with torch.no_grad():
            a = agent.actor_mean(torch.as_tensor(n)).numpy()
        obs, r, term, trunc, info = env.step(a)
        frames.append(env.render())              # rgb_array HxWx3
        ep_len += 1
        if term or trunc:
            break
    env.close()
    if not frames:
        print("no frames"); return
    h, w = frames[0].shape[:2]
    vw = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for f in frames:
        vw.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
    vw.release()
    # also drop a representative still frame next to the mp4
    png = out_path.rsplit(".", 1)[0] + ".png"
    cv2.imwrite(png, cv2.cvtColor(frames[len(frames) // 2], cv2.COLOR_RGB2BGR))
    print(f"recorded {len(frames)} frames (ep_len {ep_len}) {w}x{h} -> {out_path}")
    print(f"still frame -> {png}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--max-steps", type=int, default=1000)
    p.add_argument("--fps", type=int, default=50)
    a = p.parse_args()
    record(a.ckpt, a.out, a.max_steps, a.fps)
