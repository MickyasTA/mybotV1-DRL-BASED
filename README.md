# mybotV1 — Wheel‑Legged Bipedal Robot, Reinforcement Learning

A **two‑wheeled bipedal (wheel‑legged) robot** that learns to **balance on its wheels**
with a **from‑scratch PPO** (no Stable‑Baselines3 / no RL library). The robot has two
independent legs (3‑DOF hip + knee each) ending in driven wheels; the staged goal is:

1. **Stage 1 — balance in place** on the wheels — ✅ **done** (see below).
2. **Stage 2+ — walk / climb** with the legs *while* the wheels turn — the legs are
   already revolute + individually actuated, ready to add to the policy.

> ## ✅ Result: the robot fully balances
> Trained in **MuJoCo**, the policy holds a **full 20‑second balance** (ep_len 1000, the
> episode cap) — upright on its wheels, legs held rigid, realistic contacts/friction/
> actuators. Watch `training_results/mujoco_balance.mp4`, or see the still below.
> Training went from falling in ~0.3 s (ep_len 16) to a full balance in ~3 M steps
> (~1 hr on CPU at ~850 steps/s, fully headless — no GPU, no display).

---

## Two simulator backends

| | **MuJoCo** (primary — use this) | **Gazebo Classic + ROS 2** (legacy) |
|---|---|---|
| Holds the load‑bearing legs | ✅ 0.007 rad under full body load | ❌ collapse to ~2.45 rad instantly |
| Realistic actuators / friction / jumping | ✅ native | ⚠️ only via unrealistic hacks |
| Balance learning | ✅ **real**, converges to ep_len 1000 | ⚠️ fake (robot sat) or WIP |
| Speed / deps | ✅ ~850 sps, headless, `pip install mujoco` | 150–350 sps, needs ROS 2 + Xvfb |

**Why the split:** Gazebo Classic cannot hold this robot's load‑bearing, chained revolute
leg joints — they buckle to a fixed collapsed pose the instant physics runs and then
kinematically lock (confirmed exhaustively — see *Key findings #1*). MuJoCo's constrained
dynamics hold them realistically, so training moved to MuJoCo. The Gazebo/ROS 2 stack is
kept for integration/reference; see `CLAUDE.md` for its details.

---

## 1. What's in here

```
mybotV1-DRL-BASED/
├── README.md  CLAUDE.md            ← this file / working guide + hard‑won findings
├── requirements.txt                ← pip deps
└── src/urdf_and_meshes_of_the_robot/
    ├── meshes/                      ← STL meshes (shared by both backends)
    ├── urdf/                        ← robot xacro/urdf (mybot_geometry, mybot_balance)
    ├── mujoco/mybot.xml             ← ★ MuJoCo model (floating base, floor, actuators, IMU)
    └── scripts/
        ├── ppo_balance.py           ← ★ from‑scratch PPO trainer (--env mujoco | balance | Pendulum-v1)
        ├── balance_env_mujoco.py    ← ★ MuJoCo Gymnasium balance env
        ├── mujoco_record.py         ← ★ render a checkpoint → mp4 (headless, osmesa)
        ├── mujoco_play.py           ← ★ watch a checkpoint balance LIVE (interactive viewer)
        ├── balance_env.py           ← Gazebo/ROS 2 env (legacy)
        ├── periodic_recorder.py     ← Gazebo real‑camera recorder (legacy)
        └── env.py / train_rl_ppo.py ← original SB3 baseline (reference only, unused)
```
★ = the working MuJoCo path.

---

## 2. Quick start (MuJoCo)

Any Python 3.10+ (a GPU is **not** needed — the policy is a tiny MLP and MuJoCo runs on CPU):
```bash
pip install mujoco gymnasium torch numpy opencv-python
cd src/urdf_and_meshes_of_the_robot/scripts
```

**Sanity‑check the PPO** (no robot — the algorithm alone should learn):
```bash
python ppo_balance.py --env Pendulum-v1 --total-timesteps 150000    # return rises toward ~-200
```

**Train the balancer** (writes to `training_results/mujoco_run1/`):
```bash
python ppo_balance.py --env mujoco --total-timesteps 3000000 \
    --target-kl 0.02 --ent-coef 0.005 --eval-interval 10 \
    --save-interval 5 --checkpoint-every 100 \
    --run-dir ../../../training_results/mujoco_run1
```

**Record a video** of the best policy (offscreen — no display needed):
```bash
MUJOCO_GL=osmesa python mujoco_record.py \
    --ckpt ../../../training_results/mujoco_run1/model_best.pth \
    --out  ../../../training_results/mujoco_balance.mp4
```

**Watch it live** (interactive window — needs a display, e.g. WSLg's `:0`):
```bash
DISPLAY=:0 python mujoco_play.py --ckpt ../../../training_results/mujoco_run1/model_best.pth
```

---

## 3. The RL setup

**Model** (`mujoco/mybot.xml`, converted from the URDF): floating‑base robot on a floor,
timestep 2 ms, `implicitfast` integrator.
- **8 leg joints** — revolute, held at the 0 stance by **position servos** (kp≈150,
  ±40 Nm). Rigid enough to make the body a real inverted pendulum, still individually
  actuated (Stage 2 adds them to the policy for walking).
- **2 wheels** — **torque/motor** actuators, ±20 Nm (what the RL policy commands).
- Realistic joint damping/friction; wheel cylinders with high‑friction rolling contact;
  IMU sensors (framequat + gyro + accelerometer) for future sim‑to‑real.

**Env** (`balance_env_mujoco.py`, Gymnasium): 50 Hz control.
- **Obs** (69‑d, 3‑frame history): base roll/pitch + angular rate, wheel velocities,
  base velocity, leg positions/velocities, previous action.
- **Action**: 2 wheel torques in `[-1,1]` (scaled to ±20 Nm). Legs held at 0.
- **Reward**: `+1 alive − 2·tilt² − 0.05·drift² − 0.15·yaw_rate² − 0.01·‖a‖²`; terminate
  when tilt > 0.6 rad. The **yaw‑rate term** is what stops the robot spinning in place —
  without it the policy balances but rotates ~230°/s (≈13 turns per 20 s episode); with it
  the robot holds its heading to within ~4° (mean yaw‑rate ~18°/s) while still balancing
  the full 1000 steps.

**PPO** (`ppo_balance.py`, hand‑written): ActorCritic MLP + learnable log‑std, GAE,
clipped surrogate, running‑mean‑std obs normalization, reward normalization. Stabilized
with **`--target-kl 0.02`** (early‑stops the epoch loop — without it KL blows up to
0.3–1.0 and the policy degrades) and a small **`--ent-coef 0.005`** (prevents entropy
collapse). Standard: lr 3e‑4, n_steps 2048, batch 64, n_epochs 10, γ 0.99, gae‑λ 0.95.

---

## 4. Results layout & save scheme

Each run writes a self‑describing folder (`--run-dir`):
```
training_results/mujoco_run1/
├── model_best.pth          ← best deterministic‑eval policy   (use this for videos/deploy)
├── model_latest.pth        ← frequent resume copy (every --save-interval updates)
├── model_episode_<N>.pth   ← numbered checkpoint every --checkpoint-every EPISODES (100)
├── model_final.pth         ← end of training
├── metrics/ …              ← per‑episode CSV, summary JSON, TensorBoard, PNG graphs
└── train.log
```
Judge balancing by **`model_best`'s eval / recorded ep_len**, not the noisy raw training
`ep_len`. At convergence `model_best` balances the full 1000‑step episode.

---

## 5. Key findings (hard‑won — please keep these)

1. **Gazebo Classic cannot hold load‑bearing revolute legs → we moved to MuJoCo.** The
   leg joints buckle to ~2.45 rad the instant physics runs (before any controller
   activates) and kinematically lock. Confirmed dead across **every** lever: `position`
   (SetPosition teleport), `position_pid` (PID→SetForce, gains 200–800), `effort`+PD,
   joint `damping` 10→8000 & `friction` 100, paused‑start, `reset_simulation`; there's no
   `set_model_configuration` service in ROS 2 Gazebo to straighten them. MuJoCo holds the
   same joints at **0.007 rad** under full body load. This is a Gazebo‑Classic limitation
   with articulated load‑bearing chains, not a tuning issue.
2. **PPO stability needs a KL cap.** Without `--target-kl`, KL blew up to 0.3–1.0, the
   policy over‑updated and *degraded* (deterministic eval balanced 67 steps while training
   averaged ~700). `--target-kl 0.02` + `--ent-coef 0.005` → smooth convergence to a full
   1000‑step balance.
3. **Save scheme is decoupled**: `model_latest` (frequent) is separate from the numbered
   `model_episode_<N>` (every N **episodes**, not every update) and `model_best` (best
   eval, requires `--eval-interval > 0`).
4. **MuJoCo rendering is trivial vs Gazebo**: offscreen video via `MUJOCO_GL=osmesa`
   (CPU, headless, no display, no Xvfb, no ROS); live interactive viewer via
   `mujoco.viewer` (needs a display). Add `<visual><global offwidth/offheight></visual>`
   to the model for >640‑px offscreen frames.
5. **(Legacy, Gazebo)** the original signal bug (`/gazebo/model_states` wedging → read the
   **IMU** instead), the ODE physics constraints world1 needs, and the **Xvfb** trick for
   the Gazebo camera recorder are documented in `CLAUDE.md`.

---

## 6. Roadmap — curriculum

Each stage builds on the previous policy (curriculum learning):

- **S1 — balance in place:** ✅ done (MuJoCo, ep_len 1000 / 20 s). *Refinement:* the
  first balancer stayed up but **spun/circled in place** (the reward penalized tilt and
  x‑y drift but not yaw, so opposite wheel torques let it rotate freely). Added a
  **yaw‑rate penalty** (`−0.15·wz²`) so it station‑keeps a stable heading instead of
  circling — the first step toward goal‑driven control.
- **S1.5 — goal‑driven station‑keeping / heading:** hold a **commanded heading and x‑y
  position** (put the heading/position error in the observation + a tracking reward) — a
  clean stepping stone from "don't spin" to "go where told" before adding the legs.
- **S2 — walking:** actuate the legs (already revolute) by adding leg targets to the
  policy's action. First a **fixed gait along a specified trajectory**, then
  **goal‑conditioned walking to a target point** — add the goal (direction + distance) to
  the observation and a distance‑to‑goal / velocity‑tracking reward, with wheels and legs
  coordinated.
- **S3 — terrain:** **climb stairs** and **walk over rough terrain** — add stairs / a
  height‑field to the MuJoCo scene plus terrain‑aware perception (IMU + a height/heightmap
  or depth channel) and foot‑contact reward shaping. *MuJoCo handles these load‑bearing,
  contact‑rich dynamics realistically — the whole reason we left Gazebo Classic.*
- **S4 — sim‑to‑real:** domain randomization (mass / friction / actuator latency) and
  switch the observation from privileged base state to **IMU‑only** (the framequat / gyro
  / accelerometer sensors are already in `mybot.xml`).
