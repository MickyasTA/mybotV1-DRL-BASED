#!/usr/bin/env python3
"""Control: original Stage-2 policy in its own env + leg-mean saturation stats."""
import numpy as np
import torch

from ppo_balance import ActorCritic, RunningMeanStd
from nav_env_mujoco import GoalNavEnv

CK = ("/mnt/c/Users/mtasfaw/Documents/personal/mybotV1-DRL-BASED/"
      "training_results/nav3b_goal/model_best.pth")
ck = torch.load(CK, map_location="cpu", weights_only=False)
agent = ActorCritic(117, 10)
rms = RunningMeanStd((117,))
agent.load_state_dict(ck["model"]); rms.load_state_dict(ck["obs_rms"]); agent.eval()

env = GoalNavEnv(seed=0)
sat, tot = 0, 0
for ep in range(3):
    obs, _ = env.reset(seed=10 + ep)
    t, info = 0, {}
    for t in range(1000):
        n = rms.normalize(obs).astype(np.float32)
        with torch.no_grad():
            m = agent.actor_mean(torch.as_tensor(n)).numpy()
        sat += int(np.sum(np.abs(m[2:]) > 1.0)); tot += 8
        obs, r, term, trunc, info = env.step(m)
        if term or trunc:
            break
    print(f"control ep{ep}: steps={t+1} tilt={info['tilt']:.2f} dist={info['dist']:.2f}"
          + ("  [fell]" if term else "  [full]"))
print(f"leg-mean saturation (|m|>1): {100.0*sat/max(tot,1):.1f}% of leg commands")
env.close()
