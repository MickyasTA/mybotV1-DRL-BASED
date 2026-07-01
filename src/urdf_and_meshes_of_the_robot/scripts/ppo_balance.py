#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ppo_balance.py — Proximal Policy Optimization, written from scratch in PyTorch.

NO Stable-Baselines3, NO external RL library. Just torch + numpy + gymnasium.
This file is meant to be READ: every part of PPO is here and commented:

    * ActorCritic       : MLP policy (Gaussian) + value network
    * RunningMeanStd    : online observation normalization (our hand-written
                          equivalent of SB3's VecNormalize)
    * collect_rollout   : run the policy for n_steps, store transitions
    * compute_gae       : Generalized Advantage Estimation (GAE-lambda)
    * ppo_update        : clipped surrogate loss + value loss + entropy bonus,
                          optimized over several epochs of minibatches
    * train             : the outer loop, logging, checkpointing, evaluation

Two ways to run it
------------------
1) Validate the ALGORITHM on a standard task (no ROS / no Gazebo needed):
       python ppo_balance.py --env Pendulum-v1 --total-timesteps 150000
   A correct PPO drives the average return from ~ -1200 (random) up toward ~ -200.

2) Train the robot to self-balance (Gazebo must be running, see balance.launch.py):
       python ppo_balance.py --env balance --total-timesteps 2000000
   Watch the episode length grow (robot stays up longer) in the CSV/TensorBoard logs.

Design choices follow the deep-research findings: normalize observations, normalize
advantages, symmetric actions, tuned-ish PPO hyperparameters.
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal

try:
    from torch.utils.tensorboard import SummaryWriter
    _HAVE_TB = True
except Exception:
    _HAVE_TB = False

try:
    # results/eval layout mirroring collision_avoidance_system/models/detour_runs:
    # TensorBoard runs/ + live PNG graphs/, per-episode CSV + summary JSON.
    from tb_plot_writer import TBPlotWriter
    from metrics_logger import MetricsLogger
    _HAVE_METRICS = True
except Exception:
    _HAVE_METRICS = False

try:
    # REAL Gazebo footage of eval episodes via the headless camera sensor
    # (Xvfb + software GL). Falls back to the FK reconstruction if unavailable.
    from camera_recorder import CameraRecorder
    _HAVE_CAMREC = True
except Exception:
    _HAVE_CAMREC = False

try:
    # fallback MP4 "recording" via FK render (used only if the camera isn't up)
    from record_episode import EpisodeRecorder
    _HAVE_RECORDER = True
except Exception:
    _HAVE_RECORDER = False


# ======================================================================== #
#  Observation normalization (running mean/std, Welford parallel variance) #
# ======================================================================== #
class RunningMeanStd:
    def __init__(self, shape, epsilon=1e-4):
        self.mean = np.zeros(shape, dtype=np.float64)
        self.var = np.ones(shape, dtype=np.float64)
        self.count = epsilon

    def update(self, x):
        x = np.asarray(x, dtype=np.float64)
        if x.ndim == len(self.mean.shape):   # a single sample -> add batch dim
            x = x[None, ...]
        batch_mean = x.mean(axis=0)
        batch_var = x.var(axis=0)
        batch_count = x.shape[0]
        delta = batch_mean - self.mean
        total = self.count + batch_count
        self.mean = self.mean + delta * batch_count / total
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + np.square(delta) * self.count * batch_count / total
        self.var = m2 / total
        self.count = total

    def normalize(self, x, clip=10.0):
        out = (np.asarray(x, dtype=np.float64) - self.mean) / np.sqrt(self.var + 1e-8)
        return np.clip(out, -clip, clip).astype(np.float32)

    def state_dict(self):
        return {"mean": self.mean, "var": self.var, "count": self.count}

    def load_state_dict(self, sd):
        self.mean, self.var, self.count = sd["mean"], sd["var"], sd["count"]


class RewardNormalizer:
    """Scale rewards by the running std of the discounted return.

    This is the reward half of SB3's VecNormalize (CleanRL's NormalizeReward).
    Heterogeneous / large-magnitude rewards make value targets huge and
    advantages noisy; dividing by the return std keeps them O(1) and is one of
    the single biggest factors in PPO learning speed (deep-research finding).
    Raw rewards are still used for human-readable episode-return logging.
    """

    def __init__(self, gamma):
        self.gamma = gamma
        self.ret_rms = RunningMeanStd(())   # scalar statistics
        self.ret = 0.0

    def normalize(self, reward, done):
        self.ret = self.ret * self.gamma + reward
        self.ret_rms.update(np.array(self.ret))
        norm = reward / np.sqrt(self.ret_rms.var + 1e-8)
        if done:
            self.ret = 0.0
        # CLIP to [-10, 10] (CleanRL/SB3 NormalizeReward). Early on, the running
        # return-std is tiny/unstable, so the division can produce huge values ->
        # exploding value targets -> value-loss divergence -> the policy never
        # learns (the seed/sim-variance failures we saw). Clipping bounds that.
        return float(np.clip(norm, -10.0, 10.0))

    def state_dict(self):
        return {"ret_rms": self.ret_rms.state_dict(), "ret": self.ret}

    def load_state_dict(self, sd):
        self.ret_rms.load_state_dict(sd["ret_rms"])
        self.ret = sd["ret"]


# ======================================================================== #
#  Actor-Critic network                                                     #
# ======================================================================== #
def layer_init(layer, std=np.sqrt(2.0), bias=0.0):
    nn.init.orthogonal_(layer.weight, std)
    nn.init.constant_(layer.bias, bias)
    return layer


class ActorCritic(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden=(64, 64)):
        super().__init__()

        def mlp(out_std):
            layers, last = [], obs_dim
            for h in hidden:
                layers += [layer_init(nn.Linear(last, h)), nn.Tanh()]
                last = h
            layers += [layer_init(nn.Linear(last, act_dim if out_std is None else 1),
                                  std=out_std if out_std is not None else 0.01)]
            return nn.Sequential(*layers)

        # critic outputs a scalar value; actor outputs the action mean
        self.critic = mlp(out_std=1.0)
        self.actor_mean = mlp(out_std=None)
        # state-independent log std (learnable). Init negative (std~0.6) so the
        # initial random policy explores GENTLY around the stance instead of
        # flinging this fast/unstable robot; PPO grows it if more exploration helps.
        self.actor_logstd = nn.Parameter(torch.full((1, act_dim), -0.5))

    def get_value(self, x):
        return self.critic(x).squeeze(-1)

    def get_action_and_value(self, x, action=None):
        mean = self.actor_mean(x)
        std = torch.exp(self.actor_logstd.expand_as(mean))
        dist = Normal(mean, std)
        if action is None:
            action = dist.sample()
        logprob = dist.log_prob(action).sum(-1)
        entropy = dist.entropy().sum(-1)
        value = self.critic(x).squeeze(-1)
        return action, logprob, entropy, value


# ======================================================================== #
#  Environment factory                                                      #
# ======================================================================== #
def make_env(env_name, render=False):
    if env_name == "balance":
        from balance_env import BalanceEnv          # imported lazily (needs ROS)
        return BalanceEnv(use_pause=False)          # continuous sim for fast (10-20x) training
    if env_name == "mujoco":
        from balance_env_mujoco import MujocoBalanceEnv   # MuJoCo (realistic, actuated legs)
        return MujocoBalanceEnv(render=render)
    if env_name == "mujoco_nav":
        from nav_env_mujoco import GoalNavEnv             # Stage 2b: goal nav + actuated legs
        return GoalNavEnv(render=render)
    if env_name == "mujoco_nav_bal":
        from nav_env_mujoco import GoalNavEnv             # Stage 2a: balance WITH actuated legs
        return GoalNavEnv(render=render, goal_reward=False)
    if env_name == "mujoco_obstacle":
        from nav_env_mujoco import ObstacleNavEnv         # Stage 3: obstacles + ducking
        return ObstacleNavEnv(render=render)
    if env_name == "mujoco_terrain":
        from nav_env_mujoco import TerrainNavEnv          # Stage 4: stairs + rough terrain
        return TerrainNavEnv(render=render)
    import gymnasium as gym
    return gym.make(env_name, render_mode="human" if render else None)


# ======================================================================== #
#  GAE-lambda advantage estimation                                         #
# ======================================================================== #
def compute_gae(rewards, values, dones, last_value, last_done, gamma, lam):
    n = len(rewards)
    adv = np.zeros(n, dtype=np.float32)
    last_gae = 0.0
    for t in reversed(range(n)):
        if t == n - 1:
            next_nonterminal = 1.0 - last_done
            next_value = last_value
        else:
            next_nonterminal = 1.0 - dones[t + 1]
            next_value = values[t + 1]
        delta = rewards[t] + gamma * next_value * next_nonterminal - values[t]
        last_gae = delta + gamma * lam * next_nonterminal * last_gae
        adv[t] = last_gae
    returns = adv + values
    return adv, returns


# ======================================================================== #
#  Training                                                                #
# ======================================================================== #
def train(args):
    device = torch.device(args.device)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    env = make_env(args.env)
    obs_dim = int(np.prod(env.observation_space.shape))
    act_dim = int(np.prod(env.action_space.shape))
    print(f"[ppo] env={args.env}  obs_dim={obs_dim}  act_dim={act_dim}  device={device}")

    agent = ActorCritic(obs_dim, act_dim, hidden=tuple(args.hidden)).to(device)
    optimizer = torch.optim.Adam(agent.parameters(), lr=args.lr, eps=1e-5)
    obs_rms = RunningMeanStd((obs_dim,))
    rew_norm = RewardNormalizer(args.gamma) if args.norm_reward else None

    start_update = 0
    resume_episodes = 0
    resume_milestone = None
    resume_step = None
    if args.load and os.path.isfile(args.load):
        ckpt = torch.load(args.load, map_location=device)
        agent.load_state_dict(ckpt["model"])
        obs_rms.load_state_dict(ckpt["obs_rms"])
        if rew_norm is not None and ckpt.get("rew_norm") is not None:
            rew_norm.load_state_dict(ckpt["rew_norm"])
        start_update = ckpt.get("update", 0)
        resume_episodes = ckpt.get("total_episodes", 0)
        resume_milestone = ckpt.get("next_milestone", None)
        resume_step = ckpt.get("global_step", None)
        print(f"[ppo] resumed from {args.load} (update {start_update}, "
              f"episodes {resume_episodes}, next_milestone {resume_milestone})")

    # ---- run-folder output layout (mirrors detour_runs/run<N>/) ----
    #   <run_dir>/model_episode_<N>.pth, model_best.pth, model_final.pth, model_latest.pth
    #   <run_dir>/videos/ep<N>.mp4
    #   <run_dir>/metrics/{episodes_<sess>.csv, summary_<sess>.json, runs/<sess>/, graphs/}
    session_id = args.session_id or time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.run_dir
    metrics_dir = os.path.join(run_dir, "metrics")
    graphs_dir = os.path.join(metrics_dir, "graphs")
    videos_dir = os.path.join(run_dir, "videos")
    tb_dir = os.path.join(metrics_dir, "runs", session_id)
    for d in (run_dir, metrics_dir, graphs_dir, videos_dir, tb_dir):
        os.makedirs(d, exist_ok=True)
    os.makedirs(args.save_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)

    # TBPlotWriter (TensorBoard event files + live PNG graphs) when available,
    # else a plain SummaryWriter, else no logging.
    if _HAVE_METRICS and args.tensorboard:
        writer = TBPlotWriter(log_dir=tb_dir, graph_dir=graphs_dir)
    elif _HAVE_TB and args.tensorboard:
        writer = SummaryWriter(tb_dir)
    else:
        writer = None
    # per-episode CSV + summary JSON
    mlog = MetricsLogger(metrics_dir, session_id) if _HAVE_METRICS else None
    if mlog:
        mlog.set_start_time(time.time())

    # rollout storage
    N = args.n_steps
    obs_buf = np.zeros((N, obs_dim), dtype=np.float32)   # NORMALIZED obs
    act_buf = np.zeros((N, act_dim), dtype=np.float32)
    logp_buf = np.zeros(N, dtype=np.float32)
    rew_buf = np.zeros(N, dtype=np.float32)
    done_buf = np.zeros(N, dtype=np.float32)
    val_buf = np.zeros(N, dtype=np.float32)

    num_updates = args.total_timesteps // N
    best_eval = -float("inf")
    _last_ckpt_ep = 0        # episode count of the last numbered checkpoint
    global_step = resume_step if resume_step is not None else start_update * N
    t_start = time.time()

    # reset & normalize first observation
    raw_obs, _ = env.reset(seed=args.seed)
    obs_rms.update(raw_obs)
    next_obs = obs_rms.normalize(raw_obs)
    next_done = 0.0
    ep_returns, ep_lengths = [], []
    cur_ret, cur_len = 0.0, 0
    # per-episode accumulators for MetricsLogger (tilt/effort/duration)
    ep_wall0 = time.time()
    ep_tilt_sum = 0.0
    ep_tilt_max = 0.0
    ep_eff_sum = 0.0

    # ---- episode-based training + milestone eval/recording setup ----
    episode_mode = bool(args.max_episodes and args.max_episodes > 0)
    total_episodes = resume_episodes
    next_milestone = (resume_milestone if resume_milestone is not None
                      else (args.record_first if (episode_mode and args.record_first > 0) else None))
    recorder = None
    cam = None
    if episode_mode:
        os.makedirs(args.milestone_dir, exist_ok=True)
        if args.env == "balance" and _HAVE_RECORDER:
            try:
                recorder = EpisodeRecorder(args.mesh_dir)
            except Exception as e:
                print(f"[record] recorder init failed ({e}); milestones will be eval-only")
        if args.env == "balance" and _HAVE_CAMREC:
            try:
                cam = CameraRecorder()          # real Gazebo footage (FK render is the fallback)
            except Exception as e:
                print(f"[record] camera recorder init failed ({e}); using FK render fallback")
        print(f"[ppo] episode mode: train {args.max_episodes} episodes; milestone "
              f"(eval {args.record_eval_episodes} eps + record) at ep {args.record_first}, "
              f"then every {args.record_every}")

    update = start_update
    while (total_episodes < args.max_episodes) if episode_mode else (update < num_updates):
        # ---- 1) collect a rollout of N steps ----
        for t in range(N):
            global_step += 1
            obs_buf[t] = next_obs
            done_buf[t] = next_done
            with torch.no_grad():
                ot = torch.as_tensor(next_obs, dtype=torch.float32, device=device).unsqueeze(0)
                action, logprob, _, value = agent.get_action_and_value(ot)
            a = action.squeeze(0).cpu().numpy()
            act_buf[t] = a
            logp_buf[t] = logprob.item()
            val_buf[t] = value.item()

            raw_obs, reward, terminated, truncated, info = env.step(a)
            done = float(terminated or truncated)
            cur_ret += reward                       # raw reward for logging
            rew_buf[t] = (rew_norm.normalize(reward, bool(done))
                          if rew_norm is not None else reward)
            cur_len += 1

            # per-episode tilt/effort accumulation (for MetricsLogger)
            tilt_deg = math.degrees(math.hypot(info.get("roll", 0.0), info.get("pitch", 0.0)))
            ep_tilt_sum += tilt_deg
            ep_tilt_max = max(ep_tilt_max, tilt_deg)
            ep_eff_sum += float(np.mean(np.abs(a)))

            obs_rms.update(raw_obs)
            next_obs = obs_rms.normalize(raw_obs)
            next_done = done

            if done:
                ep_returns.append(cur_ret)
                ep_lengths.append(cur_len)
                total_episodes += 1
                # per-episode metrics row + TB scalars
                dur = time.time() - ep_wall0
                steps_ep = cur_len
                score = cur_ret
                fell = bool(terminated)
                if mlog:
                    mlog.log_episode(total_episodes, score, steps_ep, dur, fell,
                                     ep_tilt_sum / max(steps_ep, 1), ep_tilt_max,
                                     ep_eff_sum / max(steps_ep, 1), global_step, time.time())
                if writer:
                    writer.add_scalar("episode/score", score, total_episodes)
                    writer.add_scalar("episode/reward", score, total_episodes)
                    writer.add_scalar("episode/steps", steps_ep, total_episodes)
                    writer.add_scalar("episode/duration", dur, total_episodes)
                    writer.add_scalar("episode/mean_tilt_deg",
                                      ep_tilt_sum / max(steps_ep, 1), total_episodes)
                cur_ret, cur_len = 0.0, 0
                ep_wall0 = time.time(); ep_tilt_sum = ep_tilt_max = ep_eff_sum = 0.0
                raw_obs, _ = env.reset()
                obs_rms.update(raw_obs)
                next_obs = obs_rms.normalize(raw_obs)
                next_done = 0.0

        # ---- 2) bootstrap value and compute GAE ----
        with torch.no_grad():
            ot = torch.as_tensor(next_obs, dtype=torch.float32, device=device).unsqueeze(0)
            last_value = agent.get_value(ot).item()
        advantages, returns = compute_gae(
            rew_buf, val_buf, done_buf, last_value, next_done, args.gamma, args.gae_lambda)

        # ---- 3) PPO update ----
        b_obs = torch.as_tensor(obs_buf, device=device)
        b_act = torch.as_tensor(act_buf, device=device)
        b_logp = torch.as_tensor(logp_buf, device=device)
        b_adv = torch.as_tensor(advantages, device=device)
        b_ret = torch.as_tensor(returns, device=device)
        b_val = torch.as_tensor(val_buf, device=device)

        idx = np.arange(N)
        approx_kl = 0.0
        pg_loss = v_loss = ent = torch.tensor(0.0)
        for epoch in range(args.n_epochs):
            np.random.shuffle(idx)
            for start in range(0, N, args.batch_size):
                mb = idx[start:start + args.batch_size]
                _, newlogp, entropy, newval = agent.get_action_and_value(b_obs[mb], b_act[mb])
                logratio = newlogp - b_logp[mb]
                ratio = logratio.exp()

                with torch.no_grad():
                    approx_kl = ((ratio - 1.0) - logratio).mean().item()

                # normalize advantages per minibatch
                mb_adv = b_adv[mb]
                mb_adv = (mb_adv - mb_adv.mean()) / (mb_adv.std() + 1e-8)

                # clipped surrogate (policy) loss
                pg1 = -mb_adv * ratio
                pg2 = -mb_adv * torch.clamp(ratio, 1 - args.clip, 1 + args.clip)
                pg_loss = torch.max(pg1, pg2).mean()

                # clipped value loss
                if args.clip_vloss:
                    v_unclipped = (newval - b_ret[mb]) ** 2
                    v_clipped = b_val[mb] + torch.clamp(newval - b_val[mb], -args.clip, args.clip)
                    v_clipped = (v_clipped - b_ret[mb]) ** 2
                    v_loss = 0.5 * torch.max(v_unclipped, v_clipped).mean()
                else:
                    v_loss = 0.5 * ((newval - b_ret[mb]) ** 2).mean()

                ent = entropy.mean()
                loss = pg_loss - args.ent_coef * ent + args.vf_coef * v_loss

                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), args.max_grad_norm)
                optimizer.step()

            if args.target_kl is not None and approx_kl > args.target_kl:
                break  # early stop this update's epochs if policy moved too far

        # ---- 4) logging ----
        sps = int(global_step / (time.time() - t_start))
        rew_mean = float(np.mean(ep_returns[-20:])) if ep_returns else float("nan")
        len_mean = float(np.mean(ep_lengths[-20:])) if ep_lengths else float("nan")
        prog = (f"eps {total_episodes}/{args.max_episodes}" if episode_mode
                else f"upd {update+1}/{num_updates}")
        print(f"upd {update+1}  {prog}  step {global_step}  "
              f"ep_rew {rew_mean:8.2f}  ep_len {len_mean:6.1f}  "
              f"pg {pg_loss.item():+.3f}  v {v_loss.item():.3f}  "
              f"ent {ent.item():.3f}  kl {approx_kl:.4f}  {sps} sps")
        if writer:
            writer.add_scalar("charts/ep_rew_mean", rew_mean, global_step)
            writer.add_scalar("charts/ep_len_mean", len_mean, global_step)
            writer.add_scalar("loss/policy", pg_loss.item(), global_step)
            writer.add_scalar("loss/value", v_loss.item(), global_step)
            writer.add_scalar("loss/entropy", ent.item(), global_step)
            writer.add_scalar("loss/kl", approx_kl, global_step)
            writer.add_scalar("charts/sps", sps, global_step)

        # ---- 4b) early bail-out for a non-learning seed -------------------
        # Learning is seed/sim-variance sensitive. A seed that hasn't started
        # learning by ~half the episode budget won't converge in time, so exit
        # FAST (code 43) and let the wrapper retry a fresh seed -- this avoids
        # burning the remaining milestone evals on a dead run. A real learner is
        # already at ep_len >> 50 by this point.
        if (episode_mode and args.stuck_check_episodes > 0
                and total_episodes >= args.stuck_check_episodes
                and len_mean == len_mean and len_mean < args.stuck_min_len):
            print(f"[ppo] NOT LEARNING (ep {total_episodes}, ep_len {len_mean:.1f} "
                  f"< {args.stuck_min_len}) -> exit 43 for fresh-seed restart")
            try:
                if writer:
                    writer.close()
            except Exception:
                pass
            sys.exit(43)

        # ---- 5) checkpoint + periodic eval ----
        # Save scheme: model_latest.pth (frequent resume copy), model_best.pth (best eval),
        # and a numbered model_episode_<N>.pth every --checkpoint-every EPISODES.
        extra = {"total_episodes": total_episodes, "next_milestone": next_milestone,
                 "global_step": global_step}
        if (update + 1) % args.save_interval == 0:
            _save(agent, obs_rms, rew_norm, update + 1, args,
                  os.path.join(run_dir, "model_latest.pth"), extra=extra)
            if mlog:
                mlog.write_summary(time.time(), total_episodes)
        if total_episodes >= _last_ckpt_ep + args.checkpoint_every:
            _last_ckpt_ep = (total_episodes // args.checkpoint_every) * args.checkpoint_every
            _save(agent, obs_rms, rew_norm, update + 1, args,
                  os.path.join(run_dir, f"model_episode_{total_episodes}.pth"), extra=extra)
        if args.eval_interval and (update + 1) % args.eval_interval == 0:
            eval_ret = evaluate(agent, obs_rms, args, episodes=args.eval_episodes, device=device)
            print(f"    [eval] mean return over {args.eval_episodes} eps: {eval_ret:.2f}")
            if writer:
                writer.add_scalar("charts/eval_return", eval_ret, global_step)
            if eval_ret > best_eval:
                best_eval = eval_ret
                _save(agent, obs_rms, rew_norm, update + 1, args,
                      os.path.join(run_dir, "model_best.pth"))

        # ---- 6) milestone: eval N deterministic episodes + record an MP4 ----
        if episode_mode and next_milestone is not None:
            while next_milestone is not None and total_episodes >= next_milestone:
                eval_and_record(agent, obs_rms, env, recorder, args, next_milestone, device,
                                writer, cam=cam)
                # the eval stepped the env; start the next rollout from a clean reset
                raw_obs, _ = env.reset()
                obs_rms.update(raw_obs)
                next_obs = obs_rms.normalize(raw_obs)
                next_done = 0.0
                cur_ret, cur_len = 0.0, 0
                ep_wall0 = time.time(); ep_tilt_sum = ep_tilt_max = ep_eff_sum = 0.0
                next_milestone += args.record_every
                if next_milestone > args.max_episodes:
                    next_milestone = None
        update += 1

    _save(agent, obs_rms, rew_norm, update, args, os.path.join(run_dir, "model_final.pth"))
    if mlog:
        mlog.write_summary(time.time(), total_episodes)
    if episode_mode:   # final eval+record on the finished policy
        try:
            eval_and_record(agent, obs_rms, env, recorder, args, total_episodes, device,
                            writer, cam=cam)
        except Exception as e:
            print(f"[record] final eval/record failed: {e}")
    if cam is not None:
        cam.close()
    if writer:
        writer.close()
    env.close()
    print(f"[ppo] done. best eval return = {best_eval:.2f}")


def _save(agent, obs_rms, rew_norm, update, args, path, extra=None):
    data = {
        "model": agent.state_dict(),
        "obs_rms": obs_rms.state_dict(),
        "rew_norm": rew_norm.state_dict() if rew_norm is not None else None,
        "update": update,
        "args": vars(args),
        # obs/act dims so later curriculum stages can detect a layout change and
        # warm-start (weight-surgery) instead of failing load_state_dict.
        "obs_dim": int(np.prod(obs_rms.mean.shape)),
        "act_dim": int(agent.actor_logstd.shape[1]),
    }
    if extra:                       # episode/milestone state for resilient resume
        data.update(extra)
    torch.save(data, path)


def evaluate(agent, obs_rms, args, episodes, device, render=False):
    """Deterministic rollout (use the action mean, no sampling)."""
    env = make_env(args.env, render=render)
    rets = []
    for _ in range(episodes):
        raw, _ = env.reset()
        done = False
        ret = 0.0
        while not done:
            obs = obs_rms.normalize(raw)
            with torch.no_grad():
                ot = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
                mean = agent.actor_mean(ot)
            raw, r, term, trunc, _ = env.step(mean.squeeze(0).cpu().numpy())
            ret += r
            done = term or trunc
        rets.append(ret)
    env.close()
    return float(np.mean(rets))


def eval_and_record(agent, obs_rms, env, recorder, args, milestone_ep, device, writer=None,
                    cam=None, first=False):
    """Milestone hook: run `--record-eval-episodes` deterministic episodes on the
    EXISTING env (reusing the one gzserver), record the 1st episode to MP4, and log
    an eval summary. PREFERS real Gazebo camera footage (CameraRecorder); falls back
    to the FK reconstruction (EpisodeRecorder) if the camera isn't available.
    Returns (mean_ret, mean_len)."""
    metrics_dir = os.path.join(args.run_dir, "metrics")
    videos_dir = os.path.join(args.run_dir, "videos")
    os.makedirs(metrics_dir, exist_ok=True)
    os.makedirs(videos_dir, exist_ok=True)
    # the FIRST milestone file is named early_to_ep<N>.mp4
    first_milestone = first or (milestone_ep == args.record_first)
    vid = os.path.join(videos_dir,
                       ("early_to_ep%05d.mp4" % milestone_ep) if first_milestone
                       else ("ep%05d.mp4" % milestone_ep))

    rets, lens, traj = [], [], []
    video = None
    n_eval = args.record_eval_episodes
    print(f"    [milestone ep {milestone_ep}] evaluating {n_eval} episodes (deterministic)...")
    for ep in range(n_eval):
        raw, _ = env.reset()
        done, ret, length = False, 0.0, 0
        record_this = (ep == 0)
        # prefer real Gazebo camera footage for the recorded episode
        cam_active = False
        if record_this and cam is not None:
            try:
                cam_active = cam.start(vid, fps=30)
            except Exception as e:
                print(f"    [record] camera start failed: {e}")
                cam_active = False
        while not done:
            obs = obs_rms.normalize(raw)
            with torch.no_grad():
                ot = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
                a = agent.actor_mean(ot).squeeze(0).cpu().numpy()   # mean action (no sampling)
            raw, r, term, trunc, info = env.step(a)
            ret += r; length += 1; done = term or trunc
            if record_this and cam_active:
                cam.pump()                        # capture one camera frame per step
            elif record_this and hasattr(env, "_base_pos"):
                with env._lock:                   # FK fallback: stash the pose trajectory
                    x, y, z = env._base_pos
                    roll, pitch, yaw = env._base_rpy
                    wl = env._joint_pos.get("Left_joint", 0.0)
                    wr = env._joint_pos.get("Right_joint", 0.0)
                traj.append((x, y, z, roll, pitch, yaw, wl, wr))
        if record_this and cam_active:
            video = cam.finish()
        rets.append(ret); lens.append(length)
    mret, mlen = float(np.mean(rets)), float(np.mean(lens))

    # if the camera wasn't used (or produced no frames), render the same vid path
    # from the FK trajectory fallback
    if video is None and recorder is not None and traj:
        try:
            video = recorder.render(traj, vid,
                                    title=f"mybot balance @ episode {milestone_ep}  "
                                          f"(eval mean_len={mlen:.0f})",
                                    max_frames=args.record_frames)
        except Exception as e:
            print(f"    [record] video render failed: {e}")

    with open(os.path.join(metrics_dir, f"eval_summary_ep{milestone_ep:05d}.txt"), "w") as f:
        f.write(f"milestone_episode = {milestone_ep}\n")
        f.write(f"eval_episodes     = {n_eval}\n")
        f.write(f"mean_return       = {mret:.3f}\n")
        f.write(f"mean_length       = {mlen:.1f}\n")
        f.write(f"return  min/max   = {min(rets):.2f} / {max(rets):.2f}\n")
        f.write(f"length  min/max   = {min(lens)} / {max(lens)}\n")
        f.write(f"video             = {video}\n")
    ecsv = os.path.join(metrics_dir, "eval.csv")
    new = (not os.path.isfile(ecsv)) or os.stat(ecsv).st_size == 0
    with open(ecsv, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["milestone_episode", "eval_episodes", "mean_return", "mean_length"])
        w.writerow([milestone_ep, n_eval, mret, mlen])
    if writer:
        writer.add_scalar("eval/mean_return", mret, milestone_ep)
        writer.add_scalar("eval/mean_length", mlen, milestone_ep)
    print(f"    [milestone ep {milestone_ep}] mean_return={mret:.2f} mean_len={mlen:.1f}  "
          f"-> {video if video else '(no video)'}")
    return mret, mlen


def play(args):
    """Load a checkpoint and run the deterministic policy (e.g. with Gazebo GUI)."""
    device = torch.device(args.device)
    env = make_env(args.env, render=True)
    obs_dim = int(np.prod(env.observation_space.shape))
    act_dim = int(np.prod(env.action_space.shape))
    agent = ActorCritic(obs_dim, act_dim, hidden=tuple(args.hidden)).to(device)
    obs_rms = RunningMeanStd((obs_dim,))
    ckpt = torch.load(args.load, map_location=device)
    agent.load_state_dict(ckpt["model"])
    obs_rms.load_state_dict(ckpt["obs_rms"])
    raw, _ = env.reset()
    ret = 0.0
    while True:
        obs = obs_rms.normalize(raw)
        with torch.no_grad():
            ot = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
            mean = agent.actor_mean(ot)
        raw, r, term, trunc, _ = env.step(mean.squeeze(0).cpu().numpy())
        ret += r
        if term or trunc:
            print(f"episode return {ret:.2f}")
            ret = 0.0
            raw, _ = env.reset()


def parse_args():
    p = argparse.ArgumentParser(description="From-scratch PPO (no SB3).")
    p.add_argument("--env", type=str, default="balance",
                   help="'balance' (Gazebo robot) or a Gymnasium id, e.g. Pendulum-v1")
    p.add_argument("--total-timesteps", type=int, default=2_000_000)
    p.add_argument("--n-steps", type=int, default=2048)
    p.add_argument("--batch-size", type=int, default=64)
    p.add_argument("--n-epochs", type=int, default=10)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    p.add_argument("--clip", type=float, default=0.2)
    p.add_argument("--ent-coef", type=float, default=0.0)
    p.add_argument("--vf-coef", type=float, default=0.5)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--max-grad-norm", type=float, default=0.5)
    p.add_argument("--target-kl", type=float, default=None)
    p.add_argument("--clip-vloss", action="store_true", default=True)
    p.add_argument("--norm-reward", dest="norm_reward", action="store_true", default=True,
                   help="normalize rewards by running return std (VecNormalize-style)")
    p.add_argument("--no-norm-reward", dest="norm_reward", action="store_false")
    p.add_argument("--hidden", type=int, nargs="+", default=[64, 64])
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--device", type=str,
                   default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--save-dir", type=str, default="./models")
    p.add_argument("--log-dir", type=str, default="./logs")
    # ---- run-folder output layout (run_dir/ holds checkpoints, videos/, metrics/) ----
    p.add_argument("--run-dir", type=str, default="./run",
                   help="run folder: checkpoints + videos/ + metrics/ (runs/, graphs/, CSV, JSON)")
    p.add_argument("--session-id", type=str, default="",
                   help="metrics session id (empty -> %Y%m%d_%H%M%S set in train())")
    p.add_argument("--checkpoint-every", type=int, default=50,
                   help="checkpoint cadence in EPISODES (model_episode_<N>.pth)")
    p.add_argument("--stuck-check-episodes", type=int, default=2500,
                   help="episode-mode: if not learning by this episode, exit 43 (retry fresh seed)")
    p.add_argument("--stuck-min-len", type=float, default=50.0,
                   help="ep_len_mean threshold for the stuck check")
    p.add_argument("--tensorboard", action="store_true", default=True)
    p.add_argument("--save-interval", type=int, default=5)
    p.add_argument("--eval-interval", type=int, default=0,
                   help="updates between evals (0 = off; slow for the robot env)")
    p.add_argument("--eval-episodes", type=int, default=3)
    # ---- episode-based training + milestone eval/recording ----
    p.add_argument("--max-episodes", type=int, default=0,
                   help="train until this many episodes (0 = use --total-timesteps)")
    p.add_argument("--record-first", type=int, default=100,
                   help="first milestone episode at which to eval+record")
    p.add_argument("--record-every", type=int, default=400,
                   help="after the first, milestone every N episodes")
    p.add_argument("--record-eval-episodes", type=int, default=50,
                   help="deterministic eval episodes per milestone")
    p.add_argument("--record-frames", type=int, default=160,
                   help="max video frames per milestone recording")
    p.add_argument("--milestone-dir", type=str, default="./milestones",
                   help="where per-milestone videos + eval summaries are written")
    p.add_argument("--mesh-dir", type=str,
                   default=os.path.expanduser("~/ros2_ws/src/urdf_and_meshes_of_the_robot/meshes"),
                   help="STL meshes used to render the milestone videos")
    p.add_argument("--load", type=str, default=None, help="checkpoint to resume/play")
    p.add_argument("--play", action="store_true", help="run a trained policy and exit")
    return p.parse_args()


if __name__ == "__main__":
    import sys
    args = parse_args()
    if args.play:
        assert args.load, "--play requires --load <checkpoint.pt>"
        play(args)
    else:
        try:
            train(args)
        except RuntimeError as e:
            # gazebo_ros_state wedges intermittently on this WSL setup. Exit with a
            # sentinel so the resilient wrapper restarts the sim and resumes from
            # the latest checkpoint (saved every --save-interval with episode state).
            if "WEDGED" in str(e):
                print(f"[ppo] {e}\n[ppo] exiting with code 42 for resilient restart.")
                sys.exit(42)
            raise
