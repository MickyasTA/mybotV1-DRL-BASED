#!/usr/bin/env python3
"""Compare balance + yaw behavior of two PPO checkpoints (run1 spinning vs run2 yaw-penalized).

For each checkpoint: run one deterministic 1000-step episode and report
  - ep_len         (steps survived before falling; 1000 = full 20 s balance)
  - net_heading    (deg, final-minus-initial base yaw, unwrapped) -> drift
  - total_turned   (deg, sum of |d yaw| = how much it rotated in total) -> spinning
  - mean_yawrate   (deg/s, mean |wz|) -> spin intensity
"""
import os, sys, math
import numpy as np
import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from ppo_balance import ActorCritic, RunningMeanStd            # noqa: E402
from balance_env_mujoco import MujocoBalanceEnv                # noqa: E402


def _yaw(quat):                                                # mujoco quat = [w,x,y,z]
    w, x, y, z = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rollout(ckpt, seed=0, max_steps=1000):
    env = MujocoBalanceEnv(render=False, max_steps=max_steps)
    obs_dim = env.observation_space.shape[0]
    act_dim = env.action_space.shape[0]
    agent = ActorCritic(obs_dim, act_dim)
    obs_rms = RunningMeanStd((obs_dim,))
    ck = torch.load(ckpt, map_location="cpu")
    agent.load_state_dict(ck["model"]); obs_rms.load_state_dict(ck["obs_rms"]); agent.eval()

    obs, _ = env.reset(seed=seed)
    ctrl_dt = env.sim_dt * env.n_sub
    yaws, wzs = [], []
    steps = 0
    for _ in range(max_steps):
        n = obs_rms.normalize(obs).astype(np.float32)
        with torch.no_grad():
            a = agent.actor_mean(torch.as_tensor(n)).numpy()
        obs, r, term, trunc, info = env.step(a)
        q = env.data.qpos[env._base_qadr + 3: env._base_qadr + 7]
        yaws.append(_yaw(q))
        wzs.append(abs(float(env.data.qvel[env._base_vadr + 5])))
        steps += 1
        if term or trunc:
            break
    env.close()
    uw = np.unwrap(np.array(yaws))
    net = math.degrees(uw[-1] - uw[0]) if len(uw) > 1 else 0.0
    total = math.degrees(np.sum(np.abs(np.diff(uw)))) if len(uw) > 1 else 0.0
    mean_wz = math.degrees(float(np.mean(wzs))) if wzs else 0.0
    return dict(ep_len=steps, net=net, total=total, mean_wz=mean_wz, fell=bool(term))


if __name__ == "__main__":
    RES = "/mnt/c/Users/mtasfaw/Documents/personal/mybotV1-DRL-BASED/training_results"
    for tag, path in [("run1 (old, no yaw penalty)", f"{RES}/mujoco_run1/model_best.pth"),
                      ("run2 (new, yaw penalty)",    f"{RES}/mujoco_run2/model_best.pth")]:
        if not os.path.isfile(path):
            print(f"{tag:32s}  MISSING: {path}"); continue
        r = rollout(path)
        print(f"{tag:32s}  ep_len {r['ep_len']:4d}  "
              f"net_heading {r['net']:8.1f} deg  total_turned {r['total']:8.1f} deg  "
              f"mean_yawrate {r['mean_wz']:6.1f} deg/s  {'[FELL]' if r['fell'] else '[full]'}")
