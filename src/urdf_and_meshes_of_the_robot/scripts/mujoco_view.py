#!/usr/bin/env python3
"""
ONE viewer/recorder for ANY trained checkpoint — balance, goal-nav, and every future
curriculum stage, from EITHER trainer: the PyTorch CPU path (.pt/.pth, ppo_balance.py /
ppo_parallel.py) or the JAX/MJX GPU path (.pkl, mjx_ppo.py). It auto-detects the file
format, the environment, and the obs/action dimensions from the checkpoint itself, so
the same command works everywhere, every time:

  # live interactive window (needs a display, e.g. WSLg's :0) — auto-detects .pth/.pkl
  DISPLAY=:0 python mujoco_view.py --ckpt <run>/model_best.pth

  # headless mp4 (+ a still .png) — no display needed
  MUJOCO_GL=osmesa python mujoco_view.py --ckpt <run>/model_best.pth --record clip.mp4

Live viewing HOT-RELOADS: while training keeps writing a better model_best to the same
path, the viewer notices (mtime changed) and swaps in the new weights between episodes —
so a long-running `mujoco_view.py --ckpt .../model_best.pth` window always plays the
CURRENT best policy, not a one-time snapshot. Pass --no-watch to disable this.
"""
import os
# NB: do NOT force a GL backend here. It is chosen per-mode inside play()/record()
# BEFORE mujoco is first imported: 'glfw' for the live window, 'osmesa' for headless
# mp4. (Forcing 'egl' broke the live viewer on envs with an incompatible PyOpenGL.)
import sys
import time
import pickle
import argparse
import numpy as np
import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

# map a PyTorch checkpoint's action dim -> env name, when the saved args don't name it
_ACT_TO_ENV = {2: "mujoco", 10: "mujoco_nav"}
# map a JAX/MJX checkpoint's saved env tag -> the CPU-renderable env name it matches
_JAX_ENV_TO_CPU = {"mjx_balance": "mujoco", "mjx_nav": "mujoco_nav",
                   "mjx_nav_bal": "mujoco_nav_bal"}


def _kind(path):
    return "jax" if path.lower().endswith(".pkl") else "torch"


def _jax_actor_forward(params, obs_norm):
    """flax MLP forward as plain numpy — mirrors mjx_ppo.ActorCritic (Dense_0/1 tanh
    trunk, Dense_2 mean head). No jax/flax import needed to WATCH a GPU-trained policy."""
    p = params.get("params", params)
    x = obs_norm
    for i in (0, 1):
        d = p[f"Dense_{i}"]
        x = np.tanh(x @ np.asarray(d["kernel"]) + np.asarray(d["bias"]))
    d = p["Dense_2"]
    return x @ np.asarray(d["kernel"]) + np.asarray(d["bias"])


class LivePolicy:
    """Loads a .pt/.pth (PyTorch) or .pkl (JAX/MJX) checkpoint and exposes a uniform
    act(obs)->action. reload() re-reads the SAME file (used for hot-reload while a
    trainer keeps overwriting model_best); env/dims are fixed at construction since a
    single run's checkpoint doesn't change shape between saves."""

    def __init__(self, ckpt_path, obs_dim, act_dim, kind, hidden=(64, 64)):
        self.path = ckpt_path
        self.env_kind = kind
        self.obs_dim, self.act_dim = obs_dim, act_dim
        self.mtime = 0.0
        if kind == "torch":
            from ppo_balance import ActorCritic, RunningMeanStd
            self._agent = ActorCritic(obs_dim, act_dim, hidden=hidden)
            self._rms = RunningMeanStd((obs_dim,))
        self.reload(initial=True)

    def maybe_reload(self):
        try:
            m = os.path.getmtime(self.path)
        except OSError:
            return False
        if m <= self.mtime:
            return False
        try:
            self.reload()
        except Exception as e:            # trainer may be mid-write; try again next time
            print(f"[view] reload skipped (still writing?): {e}")
            return False
        return True

    def reload(self, initial=False):
        self.mtime = os.path.getmtime(self.path)
        if self.env_kind == "torch":
            ck = torch.load(self.path, map_location="cpu", weights_only=False)
            self._agent.load_state_dict(ck["model"])
            self._rms.load_state_dict(ck["obs_rms"])
            self._agent.eval()
        else:
            with open(self.path, "rb") as f:
                ck = pickle.load(f)
            self._params = ck["params"]
            self._mean, self._var = np.asarray(ck["rms"][0]), np.asarray(ck["rms"][1])
        if not initial:
            print(f"[view] hot-reloaded {os.path.basename(self.path)} "
                  f"(mtime {time.strftime('%H:%M:%S', time.localtime(self.mtime))})")

    def act(self, obs):
        if self.env_kind == "torch":
            n = self._rms.normalize(obs).astype(np.float32)
            with torch.no_grad():
                return self._agent.actor_mean(torch.as_tensor(n)).numpy()
        n = np.clip((obs - self._mean) / np.sqrt(self._var + 1e-8), -10.0, 10.0)
        return _jax_actor_forward(self._params, n.astype(np.float32))


def _inspect_ckpt(ckpt_path):
    """Read just enough of the checkpoint to know env name + obs/act dims, WITHOUT
    building the env yet (env is built once the caller knows env_name)."""
    kind = _kind(ckpt_path)
    if kind == "torch":
        ck = torch.load(ckpt_path, map_location="cpu", weights_only=False)
        args = ck.get("args", {}) or {}
        obs = int(ck["model"]["actor_mean.0.weight"].shape[1])
        act = int(ck["model"]["actor_logstd"].shape[1])
        env_name = args.get("env")
        if env_name not in ("mujoco", "mujoco_nav", "mujoco_nav_bal",
                            "mujoco_obstacle", "mujoco_obstacle_bal", "mujoco_terrain"):
            env_name = _ACT_TO_ENV.get(act, "mujoco")
        hidden = tuple(args.get("hidden") or (64, 64))
        return kind, env_name, obs, act, hidden
    with open(ckpt_path, "rb") as f:
        ck = pickle.load(f)
    env_name = _JAX_ENV_TO_CPU.get(ck.get("env"), "mujoco")
    return kind, env_name, int(ck["obs_dim"]), int(ck["act_dim"]), tuple(ck.get("hidden", (64, 64)))


def load_ckpt_env_policy(ckpt_path, render=False):
    """Build the correct CPU-renderable env + a LivePolicy for ANY checkpoint format."""
    from ppo_balance import make_env
    kind, env_name, ck_obs, ck_act, hidden = _inspect_ckpt(ckpt_path)

    env = make_env(env_name, render=render)
    obs_dim = int(np.prod(env.observation_space.shape))
    act_dim = int(np.prod(env.action_space.shape))
    if (ck_obs, ck_act) != (obs_dim, act_dim):
        env.close()
        raise SystemExit(f"[view] checkpoint is obs{ck_obs}/act{ck_act} but env "
                         f"'{env_name}' is obs{obs_dim}/act{act_dim} — mismatch.")

    policy = LivePolicy(ckpt_path, obs_dim, act_dim, kind, hidden)
    print(f"[view] kind={kind}  env={env_name}  obs={obs_dim}  act={act_dim}  "
          f"ckpt={os.path.basename(ckpt_path)}")
    return env, policy


def _episode_summary(info, steps, dt, fell):
    tag = " [fell]" if fell else " [full]"
    extra = ""
    if isinstance(info, dict) and "dist" in info:
        extra = f"  dist_to_goal={float(info['dist']):.2f}m"
        if info.get("at_goal"):
            extra += " (at goal)"
    return f"{steps} steps ({steps * dt:.1f}s){extra}{tag}"


def play(ckpt, loops=1000, seed=None, watch=True):
    os.environ.setdefault("MUJOCO_GL", "glfw")   # live window backend (needs a display)
    env, policy = load_ckpt_env_policy(ckpt, render=True)
    dt = env.sim_dt * env.n_sub
    for ep in range(loops):
        if watch and policy.maybe_reload():
            print(f"[view] episode {ep}: playing the newly updated checkpoint")
        obs, _ = env.reset(seed=seed)
        done, steps, info = False, 0, {}
        while not done:
            obs, r, term, trunc, info = env.step(policy.act(obs))
            time.sleep(dt)
            steps += 1
            done = term or trunc
        print(f"episode {ep}: " + _episode_summary(info, steps, dt, term))
    env.close()


def record(ckpt, out, max_steps=1000, fps=50, seed=0):
    os.environ.setdefault("MUJOCO_GL", "osmesa")   # headless offscreen backend (no display)
    import cv2
    env, policy = load_ckpt_env_policy(ckpt, render=False)
    obs, _ = env.reset(seed=seed)
    frames, info = [], {}
    for _ in range(max_steps):
        obs, r, term, trunc, info = env.step(policy.act(obs))
        frames.append(env.render())
        if term or trunc:
            break
    env.close()
    if not frames:
        print("no frames"); return
    h, w = frames[0].shape[:2]
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for f in frames:
        vw.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
    vw.release()
    png = out.rsplit(".", 1)[0] + ".png"
    cv2.imwrite(png, cv2.cvtColor(frames[len(frames) // 2], cv2.COLOR_RGB2BGR))
    print(f"recorded {len(frames)} frames {w}x{h} -> {out}")
    print(f"still frame -> {png}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ckpt", required=True, help=".pt / .pth (PyTorch) or .pkl (JAX/MJX)")
    p.add_argument("--record", metavar="OUT.mp4", default=None,
                   help="record an offscreen mp4 instead of opening the live viewer")
    p.add_argument("--loops", type=int, default=1000, help="live viewer: episodes to run")
    p.add_argument("--no-watch", action="store_true",
                   help="live viewer: don't hot-reload if the checkpoint file changes")
    p.add_argument("--max-steps", type=int, default=1000)
    p.add_argument("--fps", type=int, default=50)
    p.add_argument("--seed", type=int, default=None)
    a = p.parse_args()
    if a.record:
        record(a.ckpt, a.record, a.max_steps, a.fps, a.seed if a.seed is not None else 0)
    else:
        play(a.ckpt, a.loops, a.seed, watch=not a.no_watch)
