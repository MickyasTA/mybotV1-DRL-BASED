#!/usr/bin/env python3
"""
Lean multiprocessing vectorized env (SubprocVecEnv) — runs N MuJoCo envs in parallel
worker processes so a 28-core box collects ~N x the experience per second while the PPO
stays sample-efficient (moderate batch). No RL library; just multiprocessing + pipes.

Each worker owns one env, auto-resets on done, and returns the fresh obs plus the
terminal reward/done in the same step (standard vectorized autoreset).
"""
import multiprocessing as mp
import numpy as np


class EnvFactory:
    """Picklable env constructor (spawn-safe — lambdas can't be pickled)."""
    def __init__(self, env_name):
        self.env_name = env_name

    def __call__(self):
        import os, sys
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from ppo_balance import make_env
        return make_env(self.env_name)


def _worker(remote, factory, seed):
    env = factory()
    env.reset(seed=seed)
    while True:
        cmd, data = remote.recv()
        if cmd == "step":
            obs, r, term, trunc, info = env.step(data)
            if term or trunc:
                info = dict(info)
                info["terminal"] = True
                obs, _ = env.reset()
            remote.send((obs, r, bool(term), bool(trunc), info))
        elif cmd == "reset":
            obs, _ = env.reset(seed=data)
            remote.send(obs)
        elif cmd == "spaces":
            remote.send((env.observation_space, env.action_space,
                         float(env.sim_dt), int(env.n_sub),
                         int(getattr(env, "_frame_dim", 0)), int(getattr(env, "k", 1)),
                         float(getattr(env, "leg_scale", 0.0))))
        elif cmd == "close":
            env.close(); remote.close(); return


class SubprocVecEnv:
    def __init__(self, env_name, num_envs, base_seed=0):
        self.num_envs = num_envs
        ctx = mp.get_context("spawn")
        self.remotes, work = zip(*[ctx.Pipe() for _ in range(num_envs)])
        self.procs = []
        for i, (wr, rem) in enumerate(zip(work, self.remotes)):
            p = ctx.Process(target=_worker,
                            args=(wr, EnvFactory(env_name), base_seed + i), daemon=True)
            p.start()
            self.procs.append(p)
            wr.close()
        self.remotes[0].send(("spaces", None))
        (self.observation_space, self.action_space, self.sim_dt, self.n_sub,
         self.frame_dim, self.k, self.leg_scale) = self.remotes[0].recv()

    def reset(self, seed=None):
        for i, r in enumerate(self.remotes):
            r.send(("reset", None if seed is None else seed + i))
        return np.stack([r.recv() for r in self.remotes]).astype(np.float32)

    def step(self, actions):
        for r, a in zip(self.remotes, actions):
            r.send(("step", np.asarray(a)))
        outs = [r.recv() for r in self.remotes]
        obs = np.stack([o[0] for o in outs]).astype(np.float32)
        rews = np.array([o[1] for o in outs], np.float32)
        terms = np.array([o[2] for o in outs], bool)
        truncs = np.array([o[3] for o in outs], bool)
        infos = [o[4] for o in outs]
        return obs, rews, terms, truncs, infos

    def close(self):
        for r in self.remotes:
            try:
                r.send(("close", None))
            except Exception:
                pass
        for p in self.procs:
            p.join(timeout=2)
