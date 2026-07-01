#!/usr/bin/env python3
"""
duck_eval.py — verify the robot DUCKS the way we want: SQUATTING (bending the pitch/
knee joints so the base sinks straight down) rather than splaying the legs sideways.

Runs deterministic episodes in ObstacleNavEnv with the low table FORCED onto the path
(p_table=1, no pillars) and reports, per episode and in summary:
  * arrived: reached the goal on the far side of the table
  * min_h:   lowest base height while in the duck zone (should drop well below stance)
  * squat:   max |Upper/Lower (pitch) joint| while ducking  -> should be LARGE
  * splay:   max |hip_yaw/roll joint| while ducking         -> should stay SMALL
  * slab_hits: steps in contact with an obstacle (should be ~0 once learned)

  python duck_eval.py --ckpt <run>/model_best.pth [--episodes 10]
"""
import os
import sys
import argparse
import numpy as np
import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from ppo_balance import ActorCritic, RunningMeanStd            # noqa: E402
from nav_env_mujoco import ObstacleNavEnv, LEG_JOINTS          # noqa: E402


def evaluate(ckpt, episodes=10, max_steps=1000):
    env = ObstacleNavEnv(p_table=1.0, n_pillars=(0, 0))
    obs_dim = int(np.prod(env.observation_space.shape))
    act_dim = int(np.prod(env.action_space.shape))
    agent = ActorCritic(obs_dim, act_dim)
    rms = RunningMeanStd((obs_dim,))
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    agent.load_state_dict(ck["model"]); rms.load_state_dict(ck["obs_rms"]); agent.eval()

    squat_q = [env._q[LEG_JOINTS[i]] for i in env._squat_idx]
    splay_q = [env._q[LEG_JOINTS[i]] for i in (0, 1, 4, 5)]     # hip yaw/roll
    n_arr, n_duck_ok = 0, 0
    for ep in range(episodes):
        obs, _ = env.reset(seed=100 + ep)
        min_h, squat, splay, hits, ducked, arrived = 9.9, 0.0, 0.0, 0, False, False
        for _ in range(max_steps):
            n = rms.normalize(obs).astype(np.float32)
            with torch.no_grad():
                a = agent.actor_mean(torch.as_tensor(n)).numpy()
            obs, r, term, trunc, info = env.step(a)
            if info["ducking"]:
                ducked = True
                min_h = min(min_h, info["height"])
                squat = max(squat, max(abs(float(env.data.qpos[q])) for q in squat_q))
                splay = max(splay, max(abs(float(env.data.qpos[q])) for q in splay_q))
            hits += int(info["collision"])
            arrived = arrived or info["at_goal"]
            if term or trunc:
                break
        n_arr += int(arrived)
        good = ducked and squat > 0.3 and splay < 0.15
        n_duck_ok += int(good)
        print(f"ep {ep}: arrived={arrived} ducked={ducked} min_h={min_h:.2f} "
              f"squat(pitch)={squat:.2f}rad splay(yaw/roll)={splay:.2f}rad "
              f"slab_hits={hits}  {'SQUAT-OK' if good else ''}")
    env.close()
    print(f"\n{n_arr}/{episodes} arrived past the table; "
          f"{n_duck_ok}/{episodes} ducked by SQUATTING (deep pitch bend, no splay)")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", required=True)
    p.add_argument("--episodes", type=int, default=10)
    a = p.parse_args()
    evaluate(a.ckpt, a.episodes)
