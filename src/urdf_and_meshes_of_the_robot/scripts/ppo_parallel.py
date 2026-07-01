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

Results layout matches ppo_balance.py exactly (the "standard", see CLAUDE.md):
    <run_dir>/model_episode_<N>.pth, model_best.pth, model_final.pth, model_latest.pth
    <run_dir>/videos/                       <- populate via mujoco_view.py --record
    <run_dir>/metrics/episodes_<sess>.csv   <- one row per completed episode (any env)
    <run_dir>/metrics/summary_<sess>.json
    <run_dir>/metrics/runs/<sess>/          <- TensorBoard event files
    <run_dir>/metrics/graphs/               <- live PNG per scalar + _overview.png
"""
import os
import math
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

try:
    from tb_plot_writer import TBPlotWriter
    from metrics_logger import MetricsLogger
    _HAVE_METRICS = True
except Exception:
    _HAVE_METRICS = False


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

    # ---- run-folder output layout (mirrors ppo_balance.py exactly, see CLAUDE.md) ----
    session_id = args.session_id or time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.run_dir
    metrics_dir = os.path.join(run_dir, "metrics")
    graphs_dir = os.path.join(metrics_dir, "graphs")
    videos_dir = os.path.join(run_dir, "videos")
    tb_dir = os.path.join(metrics_dir, "runs", session_id)
    for d in (run_dir, metrics_dir, graphs_dir, videos_dir, tb_dir):
        os.makedirs(d, exist_ok=True)

    writer = TBPlotWriter(log_dir=tb_dir, graph_dir=graphs_dir) \
        if _HAVE_METRICS and args.tensorboard else None
    mlog = MetricsLogger(metrics_dir, session_id) if _HAVE_METRICS else None
    if mlog:
        mlog.set_start_time(time.time())

    logf = open(os.path.join(run_dir, "train.log"), "w")

    def log(s):
        print(s); logf.write(s + "\n"); logf.flush()

    if args.load and os.path.isfile(args.load):     # curriculum warm-start
        ck = torch.load(args.load, map_location=dev, weights_only=False)
        ck_obs = int(ck.get("obs_dim") or ck["model"]["actor_mean.0.weight"].shape[1])
        ck_act = int(ck.get("act_dim") or ck["model"]["actor_logstd"].shape[1])
        if (ck_obs, ck_act) == (obs_dim, act_dim):
            agent.load_state_dict(ck["model"])
            obs_rms.load_state_dict(ck["obs_rms"])
            log(f"[ppo-par] warm-started from {args.load}")
        else:
            _surgery_load(agent, obs_rms, ck, obs_dim, act_dim, ck_obs, ck_act,
                          envs.frame_dim, envs.k, log)
            log(f"[ppo-par] warm-started via WEIGHT SURGERY from {args.load} "
                f"(obs {ck_obs}->{obs_dim})")

    raw = envs.reset(seed=args.seed)
    obs_rms.update(raw)
    obs = torch.as_tensor(obs_rms.normalize(raw), device=dev)
    ep_ret = np.zeros(N); ep_len = np.zeros(N)
    ep_tilt_sum = np.zeros(N); ep_tilt_max = np.zeros(N); ep_eff_sum = np.zeros(N)
    ep_wall0 = np.full(N, time.time())
    rets, lens = deque(maxlen=200), deque(maxlen=200)
    total_episodes, _last_ckpt_ep = 0, 0

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
            act_np = action.cpu().numpy()
            raw, rew, term, trunc, infos = envs.step(act_np)
            done = np.logical_or(term, trunc)
            # reward normalization (per-env discounted-return std)
            ret = ret * args.gamma + rew
            ret_rms.update(ret)
            rw[t] = torch.as_tensor(np.clip(rew / np.sqrt(ret_rms.var + 1e-8), -10, 10),
                                    device=dev, dtype=torch.float32)
            ret[done] = 0.0
            dn[t] = torch.as_tensor(done.astype(np.float32), device=dev)
            ep_ret += rew; ep_len += 1
            # per-episode tilt/effort accumulation (for MetricsLogger, mirrors ppo_balance.py)
            tilt_deg = np.array([math.degrees(info.get("tilt", 0.0)) for info in infos])
            ep_tilt_sum += tilt_deg
            ep_tilt_max = np.maximum(ep_tilt_max, tilt_deg)
            ep_eff_sum += np.mean(np.abs(act_np), axis=1)
            for i in np.nonzero(done)[0]:
                rets.append(ep_ret[i]); lens.append(ep_len[i])
                total_episodes += 1
                if mlog:
                    mlog.log_episode(total_episodes, float(ep_ret[i]), int(ep_len[i]),
                                     time.time() - ep_wall0[i], bool(term[i]),
                                     ep_tilt_sum[i] / max(ep_len[i], 1), ep_tilt_max[i],
                                     ep_eff_sum[i] / max(ep_len[i], 1), gstep, time.time())
                if writer:
                    writer.add_scalar("episode/score", float(ep_ret[i]), total_episodes)
                    writer.add_scalar("episode/reward", float(ep_ret[i]), total_episodes)
                    writer.add_scalar("episode/steps", int(ep_len[i]), total_episodes)
                    writer.add_scalar("episode/duration", time.time() - ep_wall0[i],
                                      total_episodes)
                    writer.add_scalar("episode/mean_tilt_deg",
                                      ep_tilt_sum[i] / max(ep_len[i], 1), total_episodes)
                ep_ret[i] = 0.0; ep_len[i] = 0.0
                ep_tilt_sum[i] = ep_tilt_max[i] = ep_eff_sum[i] = 0.0
                ep_wall0[i] = time.time()
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
        kl, pg_loss, v_loss, ent_val = 0.0, 0.0, 0.0, 0.0
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
                pg_loss, v_loss, ent_val = pg.item(), vloss.item(), ent.item()
            with torch.no_grad():
                kl = float(((ratio - 1) - logratio).mean())
            if args.target_kl and kl > args.target_kl:
                break

        mret = float(np.mean(rets)) if rets else 0.0
        mlen = float(np.mean(lens)) if lens else 0.0
        sps = gstep / (time.time() - t0)
        if writer:
            writer.add_scalar("charts/ep_rew_mean", mret, gstep)
            writer.add_scalar("charts/ep_len_mean", mlen, gstep)
            writer.add_scalar("loss/policy", pg_loss, gstep)
            writer.add_scalar("loss/value", v_loss, gstep)
            writer.add_scalar("loss/entropy", ent_val, gstep)
            writer.add_scalar("loss/kl", kl, gstep)
            writer.add_scalar("charts/sps", sps, gstep)
        if update % 10 == 0 or update == num_updates - 1:
            log(f"upd {update:4d}/{num_updates}  step {gstep:9d}  ep_rew {mret:8.2f}  "
                f"ep_len {mlen:7.1f}  kl {kl:.3f}  {sps:,.0f} steps/s")
        if rets and mret > best:
            best = mret
            _save(agent, obs_rms, args, obs_dim, act_dim,
                  os.path.join(run_dir, "model_best.pth"))
        if update % 20 == 0:
            _save(agent, obs_rms, args, obs_dim, act_dim,
                  os.path.join(run_dir, "model_latest.pth"))
            if mlog:
                mlog.write_summary(time.time(), total_episodes)
        # decoupled numbered checkpoint every --checkpoint-every EPISODES (not updates)
        if total_episodes >= _last_ckpt_ep + args.checkpoint_every:
            _last_ckpt_ep = (total_episodes // args.checkpoint_every) * args.checkpoint_every
            _save(agent, obs_rms, args, obs_dim, act_dim,
                  os.path.join(run_dir, f"model_episode_{total_episodes}.pth"))
    _save(agent, obs_rms, args, obs_dim, act_dim, os.path.join(run_dir, "model_final.pth"))
    if mlog:
        mlog.write_summary(time.time(), total_episodes)
    if writer:
        writer.close()
    envs.close()
    log(f"[ppo-par] done. best ep_rew={best:.2f}  {time.time()-t0:.0f}s")


def _surgery_load(agent, obs_rms, ck, obs_dim, act_dim, ck_obs, ck_act,
                  frame_dim, k, log):
    """Warm-start into a WIDER observation (curriculum stage change, e.g. Stage-2 nav
    obs117 -> Stage-3 obstacle obs183). Works because every stage keeps the previous
    per-frame layout as a PREFIX and only appends new features (rays etc.) at the end
    of each frame: old first-layer columns are copied to their new (shifted) positions,
    the brand-new feature columns start at zero (so the loaded policy initially IGNORES
    the new inputs and behaves exactly like the Stage-2 policy), and everything past
    the first layer copies verbatim. Action dim must be unchanged."""
    assert ck_act == act_dim, f"action-dim change {ck_act}->{act_dim} not supported"
    assert ck_obs % k == 0 and frame_dim * k == obs_dim, "frame layout mismatch"
    old_frame = ck_obs // k
    assert old_frame <= frame_dim, "new frame must be a superset of the old one"
    new_idx = torch.tensor([f * frame_dim + j for f in range(k) for j in range(old_frame)])
    old_idx = torch.tensor([f * old_frame + j for f in range(k) for j in range(old_frame)])

    sd_new = agent.state_dict()
    sd_old = ck["model"]
    for name in sd_new:
        wold = sd_old.get(name)
        if wold is None:
            log(f"[surgery] missing in ckpt, kept init: {name}")
        elif sd_new[name].shape == wold.shape:
            sd_new[name] = wold.clone()
        elif name.endswith(".0.weight") and sd_new[name].shape[0] == wold.shape[0]:
            w = torch.zeros_like(sd_new[name])
            w[:, new_idx] = wold[:, old_idx]
            sd_new[name] = w
        else:
            log(f"[surgery] shape mismatch, kept init: {name} "
                f"{tuple(wold.shape)} -> {tuple(sd_new[name].shape)}")
    agent.load_state_dict(sd_new)

    old_rms = ck["obs_rms"]
    mean = np.zeros(obs_dim, np.float64)
    var = np.ones(obs_dim, np.float64)
    mean[new_idx.numpy()] = np.asarray(old_rms["mean"])[old_idx.numpy()]
    var[new_idx.numpy()] = np.asarray(old_rms["var"])[old_idx.numpy()]
    obs_rms.load_state_dict({"mean": mean, "var": var, "count": old_rms["count"]})


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
    p.add_argument("--session-id", default="", help="metrics session id (empty -> timestamp)")
    p.add_argument("--checkpoint-every", type=int, default=200,
                   help="checkpoint cadence in EPISODES (model_episode_<N>.pth)")
    p.add_argument("--tensorboard", action="store_true", default=True)
    train(p.parse_args())
