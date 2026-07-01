#!/usr/bin/env python3
"""
Parallel-CPU PPO — the same hand-written PyTorch PPO as ppo_balance.py, but collecting
from N MuJoCo envs at once (vec_env.SubprocVecEnv) so a many-core box trains ~N x faster
WITHOUT the sample-efficiency loss of massively-parallel GPU PPO (moderate batch, high
update ratio). No RL library. Checkpoints are mujoco_view.py-compatible.

  python ppo_parallel.py --env mujoco_nav --num-envs 24 --total-timesteps 8000000 \
      --run-dir ../../../training_results/nav_par1

For GPU acceleration on a bigger machine use the MJX path instead (mjx_ppo.py) — kept as
a flag for desktop 4090 / cloud A100 where MJX's throughput wins.
"""
import os
import time
import argparse
from collections import deque

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal

_HERE = os.path.dirname(os.path.abspath(__file__))
import sys
sys.path.insert(0, _HERE)
from ppo_balance import ActorCritic, RunningMeanStd     # noqa: E402
from vec_env import SubprocVecEnv                         # noqa: E402


def train(args):
    dev = torch.device(args.device)
    torch.manual_seed(args.seed); np.random.seed(args.seed)
    envs = SubprocVecEnv(args.env, args.num_envs, base_seed=args.seed)
    obs_dim = int(np.prod(envs.observation_space.shape))
    act_dim = int(np.prod(envs.action_space.shape))
    N, T = args.num_envs, args.n_steps
    B = N * T
    mb = max(1, B // args.num_minibatches)

    agent = ActorCritic(obs_dim, act_dim, hidden=tuple(args.hidden)).to(dev)
    opt = torch.optim.Adam(agent.parameters(), lr=args.lr, eps=1e-5)
    obs_rms = RunningMeanStd((obs_dim,))
    ret_rms = RunningMeanStd(())            # for reward normalization
    ret = np.zeros(N, np.float64)

    o = torch.zeros((T, N, obs_dim), device=dev)
    a = torch.zeros((T, N, act_dim), device=dev)
    lg = torch.zeros((T, N), device=dev)
    vl = torch.zeros((T, N), device=dev)
    rw = torch.zeros((T, N), device=dev)
    dn = torch.zeros((T, N), device=dev)

    os.makedirs(args.run_dir, exist_ok=True)
    logf = open(os.path.join(args.run_dir, "train.log"), "w")

    def log(s):
        print(s); logf.write(s + "\n"); logf.flush()

    if args.load and os.path.isfile(args.load):     # curriculum warm-start (same dims)
        ck = torch.load(args.load, map_location=dev)
        agent.load_state_dict(ck["model"])
        obs_rms.load_state_dict(ck["obs_rms"])
        log(f"[ppo-par] warm-started from {args.load}")

    raw = envs.reset(seed=args.seed)
    obs_rms.update(raw)
    obs = torch.as_tensor(obs_rms.normalize(raw), device=dev)
    ep_ret = np.zeros(N); ep_len = np.zeros(N)
    rets, lens = deque(maxlen=200), deque(maxlen=200)

    num_updates = args.total_timesteps // B
    best, gstep, t0 = -1e9, 0, time.time()
    log(f"[ppo-par] env={args.env} envs={N} n_steps={T} batch={B} minibatch={mb} "
        f"updates={num_updates} obs={obs_dim} act={act_dim} device={args.device}")

    for update in range(num_updates):
        for t in range(T):
            o[t] = obs
            with torch.no_grad():
                mean = agent.actor_mean(obs)
                std = agent.actor_logstd.exp().expand_as(mean)
                dist = Normal(mean, std)
                action = dist.sample()
                lg[t] = dist.log_prob(action).sum(-1)
                vl[t] = agent.critic(obs).squeeze(-1)
            a[t] = action
            raw, rew, term, trunc, infos = envs.step(action.cpu().numpy())
            done = np.logical_or(term, trunc)
            # reward normalization (per-env discounted-return std)
            ret = ret * args.gamma + rew
            ret_rms.update(ret)
            rw[t] = torch.as_tensor(np.clip(rew / np.sqrt(ret_rms.var + 1e-8), -10, 10),
                                    device=dev, dtype=torch.float32)
            ret[done] = 0.0
            dn[t] = torch.as_tensor(done.astype(np.float32), device=dev)
            ep_ret += rew; ep_len += 1
            for i in np.nonzero(done)[0]:
                rets.append(ep_ret[i]); lens.append(ep_len[i])
                ep_ret[i] = 0.0; ep_len[i] = 0.0
            obs_rms.update(raw)
            obs = torch.as_tensor(obs_rms.normalize(raw), device=dev)
            gstep += N

        with torch.no_grad():
            next_val = agent.critic(obs).squeeze(-1)
        adv = torch.zeros_like(rw)
        last = torch.zeros(N, device=dev)
        for t in reversed(range(T)):
            nonterm = 1.0 - dn[t]
            nextv = next_val if t == T - 1 else vl[t + 1]
            delta = rw[t] + args.gamma * nextv * nonterm - vl[t]
            adv[t] = last = delta + args.gamma * args.gae_lambda * nonterm * last
        rets_t = adv + vl

        bo, ba = o.reshape(B, obs_dim), a.reshape(B, act_dim)
        blg, badv, bret, bval = lg.reshape(B), adv.reshape(B), rets_t.reshape(B), vl.reshape(B)
        idx = np.arange(B)
        kl = 0.0
        for epoch in range(args.epochs):
            np.random.shuffle(idx)
            for s in range(0, B, mb):
                j = idx[s:s + mb]
                mean = agent.actor_mean(bo[j])
                std = agent.actor_logstd.exp().expand_as(mean)
                dist = Normal(mean, std)
                nlg = dist.log_prob(ba[j]).sum(-1)
                ent = dist.entropy().sum(-1).mean()
                nval = agent.critic(bo[j]).squeeze(-1)
                logratio = nlg - blg[j]
                ratio = logratio.exp()
                mba = badv[j]
                mba = (mba - mba.mean()) / (mba.std() + 1e-8)
                pg = torch.max(-mba * ratio,
                               -mba * torch.clamp(ratio, 1 - args.clip, 1 + args.clip)).mean()
                vc = bval[j] + torch.clamp(nval - bval[j], -args.clip, args.clip)
                vloss = 0.5 * torch.max((nval - bret[j]) ** 2, (vc - bret[j]) ** 2).mean()
                loss = pg + args.vf * vloss - args.ent * ent
                opt.zero_grad(); loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), 0.5)
                opt.step()
            with torch.no_grad():
                kl = float(((ratio - 1) - logratio).mean())
            if args.target_kl and kl > args.target_kl:
                break

        mret = float(np.mean(rets)) if rets else 0.0
        mlen = float(np.mean(lens)) if lens else 0.0
        sps = gstep / (time.time() - t0)
        if update % 10 == 0 or update == num_updates - 1:
            log(f"upd {update:4d}/{num_updates}  step {gstep:9d}  ep_rew {mret:8.2f}  "
                f"ep_len {mlen:7.1f}  kl {kl:.3f}  {sps:,.0f} steps/s")
        if rets and mret > best:
            best = mret
            _save(agent, obs_rms, args, obs_dim, act_dim,
                  os.path.join(args.run_dir, "model_best.pth"))
        if update % 20 == 0:
            _save(agent, obs_rms, args, obs_dim, act_dim,
                  os.path.join(args.run_dir, "model_latest.pth"))
    _save(agent, obs_rms, args, obs_dim, act_dim,
          os.path.join(args.run_dir, "model_final.pth"))
    envs.close()
    log(f"[ppo-par] done. best ep_rew={best:.2f}  {time.time()-t0:.0f}s")


def _save(agent, obs_rms, args, obs_dim, act_dim, path):
    torch.save({"model": agent.state_dict(), "obs_rms": obs_rms.state_dict(),
                "rew_norm": None, "args": vars(args),
                "obs_dim": obs_dim, "act_dim": act_dim}, path)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--env", default="mujoco_nav")
    p.add_argument("--num-envs", type=int, default=24)
    p.add_argument("--n-steps", type=int, default=128)
    p.add_argument("--total-timesteps", type=int, default=8_000_000)
    p.add_argument("--epochs", type=int, default=10)
    p.add_argument("--num-minibatches", type=int, default=32)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    p.add_argument("--clip", type=float, default=0.2)
    p.add_argument("--ent", type=float, default=0.005)
    p.add_argument("--vf", type=float, default=0.5)
    p.add_argument("--target-kl", type=float, default=0.02)
    p.add_argument("--hidden", type=int, nargs="+", default=[64, 64])
    p.add_argument("--device", default="cpu")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--run-dir", default="../../../training_results/par1")
    p.add_argument("--load", default=None, help="warm-start checkpoint (same obs/act dims)")
    train(p.parse_args())
