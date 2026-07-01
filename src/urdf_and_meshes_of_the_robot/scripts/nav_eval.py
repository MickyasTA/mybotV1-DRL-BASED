#!/usr/bin/env python3
"""Direct goal-reaching evaluation for a nav checkpoint: over N episodes, report
start distance, end distance, whether it arrived (dist<goal_tol), and ep_len."""
import os, sys, argparse
import numpy as np
import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from ppo_balance import ActorCritic, RunningMeanStd
from nav_env_mujoco import GoalNavEnv


def evaluate(ckpt, episodes=10, max_steps=1000):
    env = GoalNavEnv()
    obs_dim = env.observation_space.shape[0]
    act_dim = env.action_space.shape[0]
    agent = ActorCritic(obs_dim, act_dim)
    obs_rms = RunningMeanStd((obs_dim,))
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    agent.load_state_dict(ck["model"]); obs_rms.load_state_dict(ck["obs_rms"]); agent.eval()

    arrivals, ep_lens, start_d, end_d = 0, [], [], []
    for ep in range(episodes):
        obs, _ = env.reset(seed=ep)
        d0 = float(np.hypot(*env.goal_xy))
        steps, arrived, info = 0, False, {}
        for _ in range(max_steps):
            n = obs_rms.normalize(obs).astype(np.float32)
            with torch.no_grad():
                a = agent.actor_mean(torch.as_tensor(n)).numpy()
            obs, r, term, trunc, info = env.step(a)
            steps += 1
            if info.get("at_goal"):
                arrived = True
            if term or trunc:
                break
        arrivals += int(arrived)
        ep_lens.append(steps); start_d.append(d0); end_d.append(info.get("dist", d0))
        print(f"ep {ep}: goal_dist0={d0:.2f}m  end_dist={info.get('dist', -1):.2f}m  "
              f"steps={steps}  {'ARRIVED' if arrived else 'no-arrival'}"
              f"{'  [fell]' if term else ''}")
    env.close()
    print(f"\n{arrivals}/{episodes} episodes arrived at goal  "
          f"mean_ep_len={np.mean(ep_lens):.0f}  "
          f"mean_start_dist={np.mean(start_d):.2f}m  mean_end_dist={np.mean(end_d):.2f}m")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", required=True)
    p.add_argument("--episodes", type=int, default=10)
    a = p.parse_args()
    evaluate(a.ckpt, a.episodes)
