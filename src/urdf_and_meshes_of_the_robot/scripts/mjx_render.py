#!/usr/bin/env python3
"""
Watch / record a GPU-trained (MJX / JAX) policy in the CPU MuJoCo env — no JAX needed to
view. The flax actor is a plain MLP, so we run it as a numpy forward pass and drive the
matching CPU env (balance or goal-nav), which has the SAME obs/action layout.

  # record an mp4 (headless)
  python mjx_render.py --ckpt <run>/model_best.pkl --record clip.mp4
  # watch live
  DISPLAY=:0 python mjx_render.py --ckpt <run>/model_best.pkl

Note: MJX trains with the pyramidal cone; the CPU env uses elliptic — a small sim-to-sim
gap, fine for visualization. Run this from the depth_bench env (has torch/cv2/mujoco).
"""
import os
import sys
import time
import pickle
import argparse
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

_ENV_MAP = {"mjx_nav": ("nav_env_mujoco", "GoalNavEnv"),
            "mjx_balance": ("balance_env_mujoco", "MujocoBalanceEnv")}


def _actor(params, obs_norm):
    p = params.get("params", params)
    x = obs_norm
    for i in (0, 1):                       # actor trunk Dense_0, Dense_1 (tanh)
        d = p[f"Dense_{i}"]
        x = np.tanh(x @ np.asarray(d["kernel"]) + np.asarray(d["bias"]))
    d = p["Dense_2"]                        # mean head (deterministic action)
    return x @ np.asarray(d["kernel"]) + np.asarray(d["bias"])


def make_env(env_name, render):
    mod, cls = _ENV_MAP.get(env_name, _ENV_MAP["mjx_balance"])
    m = __import__(mod)
    return getattr(m, cls)(render=render)


def run(ckpt, out=None, live=False, max_steps=1000, seed=0):
    with open(ckpt, "rb") as f:
        ck = pickle.load(f)
    params, (mean, var), env_name = ck["params"], ck["rms"], ck.get("env", "mjx_balance")
    mean, var = np.asarray(mean), np.asarray(var)
    env = make_env(env_name, render=live)
    print(f"[mjx-render] env={env_name} obs={ck['obs_dim']} act={ck['act_dim']} "
          f"{'live' if live else 'record'}")
    obs, _ = env.reset(seed=seed)
    dt = env.sim_dt * env.n_sub
    frames, steps, info = [], 0, {}
    for _ in range(max_steps):
        n = np.clip((obs - mean) / np.sqrt(var + 1e-8), -10.0, 10.0).astype(np.float32)
        obs, r, term, trunc, info = env.step(_actor(params, n))
        steps += 1
        if live:
            time.sleep(dt)
        else:
            frames.append(env.render())
        if term or trunc:
            break
    extra = f" dist={info.get('dist'):.2f}" if "dist" in info else ""
    print(f"episode: {steps} steps ({steps*dt:.1f}s){extra}" + (" [fell]" if term else " [full]"))
    env.close()
    if out and frames:
        import cv2
        h, w = frames[0].shape[:2]
        vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), 50, (w, h))
        for fr in frames:
            vw.write(cv2.cvtColor(fr, cv2.COLOR_RGB2BGR))
        vw.release()
        png = out.rsplit(".", 1)[0] + ".png"
        cv2.imwrite(png, cv2.cvtColor(frames[len(frames) // 2], cv2.COLOR_RGB2BGR))
        print(f"recorded {len(frames)} frames -> {out}\nstill -> {png}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", required=True)
    p.add_argument("--record", metavar="OUT.mp4", default=None)
    p.add_argument("--max-steps", type=int, default=1000)
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args()
    os.environ.setdefault("MUJOCO_GL", "osmesa" if a.record else "glfw")
    run(a.ckpt, a.record, live=not a.record, max_steps=a.max_steps, seed=a.seed)
