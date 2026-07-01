# mybotV1 — Wheel‑Legged Bipedal Robot, Reinforcement Learning

A **two‑wheeled bipedal (wheel‑legged) robot** that learns to **balance on its wheels**
with a **from‑scratch PPO** (no Stable‑Baselines3 / no RL library). The robot has two
independent legs (3‑DOF hip + knee each) ending in driven wheels; the staged goal is:

1. **Stage 1 — balance in place** on the wheels — ✅ **done** (see below).
2. **Stage 2+ — walk / climb** with the legs *while* the wheels turn — the legs are
   already revolute + individually actuated, ready to add to the policy.

> ## ✅ Result: the robot balances AND drives to a goal point
> **Stage 1 (balance):** trained in **MuJoCo**, the policy holds a **full 20‑second
> balance** (ep_len 1000) — upright on its wheels, legs held, realistic contacts/friction.
> **Stage 2 (goal‑navigation):** with the **8 leg joints now actuated** (no longer fixed),
> the robot drives to a randomized goal point and **stops there** — verified over 10 held‑
> out episodes: **9/10 arrived**, mean end‑distance **0.16 m** from a mean start of
> **1.47 m**, still balancing the full 1000 steps. Watch
> `training_results/mujoco_balance.mp4` (Stage 1) and
> `training_results/nav_goal_reach.mp4` (Stage 2, goal marker visible).

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
        ├── ppo_balance.py           ← ★ from‑scratch PPO, single env (--env mujoco | mujoco_nav | balance | Pendulum-v1)
        ├── ppo_parallel.py          ← ★ same PPO, N parallel MuJoCo envs (multi‑core CPU, ~5x @ 24 envs)
        ├── vec_env.py               ← multiprocessing SubprocVecEnv (no RL library)
        ├── balance_env_mujoco.py    ← ★ MuJoCo Gymnasium env — Stage 1 balance (obs69/act2)
        ├── nav_env_mujoco.py        ← ★ MuJoCo Gymnasium env — Stage 2 goal‑nav (obs117/act10, legs actuated)
        ├── nav_eval.py              ← direct goal‑reaching evaluation (arrival rate, end‑distance)
        ├── mujoco_view.py           ← ★ view/record ANY checkpoint (.pth or .pkl), auto‑detect + hot‑reload
        ├── mujoco_record.py / mujoco_play.py  ← thin shims → mujoco_view.py (kept for old commands)
        ├── mjx_env.py / mjx_nav_env.py / mjx_ppo.py / mjx_render.py  ← GPU path (MJX/JAX), see §4b
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

**Train the balancer** (single env; writes to `training_results/mujoco_run1/`):
```bash
python ppo_balance.py --env mujoco --total-timesteps 3000000 \
    --target-kl 0.02 --ent-coef 0.005 --eval-interval 10 \
    --save-interval 5 --checkpoint-every 100 \
    --run-dir ../../../training_results/mujoco_run1
```

**Train goal‑navigation** (Stage 2, legs actuated) — use `ppo_parallel.py` (N MuJoCo envs
across CPU cores; a 28‑core box does ~5x with `--num-envs 24`) and a **2‑step curriculum**:
first learn to balance *with the legs held near their stance* (no goal yet — the legs
jittering as a new 8‑D action is what topples a from‑scratch attempt), THEN warm‑start
that policy with the goal added. Same obs/action shape both stages → clean warm‑start:
```bash
# 2a: balance WITH actuated legs, no goal (station-keeps at the origin)
python ppo_parallel.py --env mujoco_nav_bal --num-envs 24 --total-timesteps 6000000 \
    --run-dir ../../../training_results/nav2a_balleg

# 2b: warm-start 2a, add the goal — drive to a randomized point and hold
python ppo_parallel.py --env mujoco_nav --num-envs 24 --total-timesteps 6000000 \
    --load ../../../training_results/nav2a_balleg/model_best.pth \
    --run-dir ../../../training_results/nav2b_goal

# verify: arrival rate + end-distance over held-out episodes
python nav_eval.py --ckpt ../../../training_results/nav2b_goal/model_best.pth --episodes 10
```

**View / record ANY checkpoint — one generalizable script** (`mujoco_view.py`). It
auto‑detects the **file format** (`.pth`/`.pt` from the PyTorch trainers, `.pkl` from the
JAX/MJX GPU trainer), the environment, and the obs/action dimensions **from the checkpoint
itself**, so the same command works for every stage and either trainer — no need to edit
the script when the architecture changes. It also picks the right GL backend automatically
(`glfw` for the live window, `osmesa` for headless mp4), so no more EGL errors.

```bash
# watch live in a window (needs a display, e.g. WSLg's :0)
DISPLAY=:0 python mujoco_view.py --ckpt ../../../training_results/mujoco_run1/model_best.pth

# record an mp4 (+ a still .png) — headless, no display needed
python mujoco_view.py --ckpt ../../../training_results/nav2b_goal/model_best.pth --record clip.mp4
```
**Live viewing hot‑reloads:** if you leave the live window open while a trainer keeps
writing an improved `model_best` to the same path, `mujoco_view.py` notices (checks the
file's mtime) and swaps in the new weights between episodes — so a long‑running window
always plays the *current* best policy, not a one‑time snapshot. Pass `--no-watch` to
disable. (`mujoco_play.py` and `mujoco_record.py` still work — they forward to
`mujoco_view.py`.)

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

**Stage‑2 goal‑nav env** (`nav_env_mujoco.py`): same base as Stage 1, plus:
- **Obs** (117‑d, 3‑frame history): all 9 base‑state terms + **all 8** leg positions/
  velocities (Stage 1 only exposed 6 hip joints) + a **base‑frame goal vector**
  `[dir_x, dir_y, heading_err, distance]` + the 10‑d previous action.
- **Action** (10‑d): 2 wheel torques + **8 leg position targets** (± a small range around
  the 0 stance — kept small so exploration jitter doesn't topple balance; widen at the
  ducking/terrain stage where the legs need real range of motion).
- **Reward** adds to the Stage‑1 terms: dense **potential‑based progress** toward the
  goal, **velocity‑toward‑goal**, **face‑the‑goal**, an **arrival bonus** paid every step
  while inside the goal radius, a **stall penalty** for standing still far from the goal,
  and — the important one — a **posture penalty** (`−0.6·Σ leg_qpos²`) that keeps the legs
  near their stance. *Why it matters:* without it, the very first goal‑nav policy solved
  balance by **splaying its legs into a wide static split** — free lateral stability, but
  not the point of actuating the legs at all. The posture penalty removed that shortcut.
- **Curriculum, not one‑shot:** actuating the legs (10‑D action) *and* adding a goal *and*
  banning the splay‑shortcut, all at once, from scratch, made the robot fall in ~5 steps
  every time — an 8‑D action jittering during early exploration is simply too destabilizing
  layered on a brand‑new task. Splitting into **2a** (balance‑with‑legs, no goal) → **2b**
  (warm‑start 2a, add the goal) fixed it: 2a converges to ep_len ~900/1000 in a few minutes,
  and 2b *keeps* that balance from step one (ep_len jumps straight to ~800 by update 10)
  while learning to navigate on top of it.

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

## 4b. Training speed — CPU vs GPU (MJX) vs parallel‑CPU

The policy is a tiny MLP; the bottleneck is **physics**, which classic MuJoCo runs on
**CPU**. So `--device cuda` on the single‑env trainer does **not** help (the GPU can't be
fed one step at a time). Three paths were tried, measured honestly on an **RTX 4090
Laptop, 28‑core host**:

| path | how | raw throughput | *wall‑clock to converge* |
|---|---|---|---|
| **CPU, 1 env** | `ppo_balance.py --device cpu` | ~850 steps/s | baseline (~30 min) |
| **GPU / MJX** | MuJoCo‑on‑GPU (JAX), 2048+ envs | ~7,000–18,000 steps/s | **≈ same or worse** |
| **Parallel CPU** (recommended here) | `ppo_parallel.py`, N MuJoCo envs across cores | ~2,400–4,000 steps/s (24 envs) | **~5–10× faster** |

**Why MJX didn't win here, despite ~10× raw throughput:** massively‑parallel PPO (2048+
envs) collects a huge batch every iteration but does far fewer gradient updates per
environment‑step than a moderate‑batch setup — it trades sample‑efficiency for
throughput. On a **laptop** GPU with this contact‑rich model, that trade is close to a
wash: balance that reaches ep_len ~900 in ~1.3 M CPU‑equivalent samples still needed
≫3 M samples on MJX to get partway there. On a **datacenter GPU (A100)** or a bare‑metal
desktop 4090, physics throughput is high enough that the trade wins decisively (this is
how MuJoCo Playground trains policies in minutes) — so the JAX/MJX stack (`mjx_env.py`,
`mjx_nav_env.py`, `mjx_ppo.py`, `mjx_render.py`, isolated `mybot_mjx` conda env, no RL
library — `flax`/`optax` only) is **kept as a flag for a bigger machine**, not removed.

**Parallel CPU (`ppo_parallel.py`) is the win on this machine**: N independent MuJoCo
envs run in worker processes (`vec_env.py`, plain `multiprocessing`, no RL library);
the PPO stays exactly as sample‑efficient as the single‑env version (moderate batch,
many gradient updates), just fed N× faster. This is what trained the Stage‑2 goal‑nav
result above.

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
- **S2 — goal‑conditioned navigation, legs actuated:** ✅ done. Legs went from held‑at‑0
  to a real 8‑D action (position targets); a base‑frame goal vector was added to the obs;
  reward adds progress/velocity‑toward‑goal/face‑goal/arrival/station‑keep terms plus a
  **posture penalty** (kills the leg‑splay shortcut a first attempt found). Trained via a
  **2‑stage curriculum** (2a balance‑with‑legs → 2b warm‑start + goal) on parallel‑CPU
  (`ppo_parallel.py`) — see §3/§4b. **Verified**: 9/10 held‑out episodes arrive at a
  randomized goal and station‑keep (mean end‑distance 0.16 m from a 1.47 m start), full
  1000‑step balance throughout.
- **S2.5 — obstacle avoidance / dodging:** exteroceptive perception (rangefinder rays or
  an offscreen depth camera) + a collision/proximity penalty, so the robot routes around
  obstacles on the way to the goal instead of assuming clear flat ground.
- **S3 — ducking + terrain:** **squat/lower the legs to pass under a low obstacle**
  (e.g. a table) — the leg *range* deliberately kept small in S2 needs widening here, now
  that there's a real reason to move them; **climb stairs** and **walk over rough
  terrain** via a height‑field/procedural stairs in the MuJoCo scene, terrain‑aware
  perception, and foot/wheel‑contact reward shaping. *MuJoCo handles these load‑bearing,
  contact‑rich dynamics realistically — the whole reason we left Gazebo Classic.*
- **S4 — sim‑to‑real:** domain randomization (mass / friction / actuator latency) and
  switch the observation from privileged base state to **IMU‑only** (the framequat / gyro
  / accelerometer sensors are already in `mybot.xml`).
