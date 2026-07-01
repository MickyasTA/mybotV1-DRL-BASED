# CLAUDE.md — mybotV1 (wheel‑legged bipedal balance RL)

Working guide for this repo. Read this before changing sim/control/RL code.

## What this is
A **two‑wheeled bipedal (wheel‑legged) robot** learning to **balance** with a
**from‑scratch PPO** (NO Stable‑Baselines3 / no RL library) in **ROS 2 Humble +
Gazebo Classic 11 on WSL2**. Staged goal: (S1) balance in place on the wheels →
(S2+) walk/climb with the legs while the wheels roll.

## Active stack (everything else is legacy/reference)
- `src/urdf_and_meshes_of_the_robot/scripts/balance_env.py` — Gymnasium env. Obs from
  the **IMU** (tilt/rate, 90° mount offset undone) + **wheel odometry**; action = 2 wheel
  efforts. Legs held at a 0 stance (see leg note below). Reset = pause → `/reset_world`
  → command → unpause; stepping is `/clock`‑paced.
- `scripts/ppo_balance.py` — from‑scratch PPO (ActorCritic MLP, GAE, clipped surrogate,
  obs RunningMeanStd, reward norm). `--env Pendulum-v1` self‑test; `--env balance` trains.
  Writes the detour‑style results layout under `--run-dir`.
- `scripts/periodic_recorder.py` — **real** headless‑Gazebo mp4 recorder (spawns a camera,
  records, deletes). `scripts/record_episode.py` — FK/matplotlib fallback render.
- `launch/balance.launch.py`, `worlds/{balance,world1}.world`, `config/{controllers,gazebo_params}.yaml`.

## Run (WSL, two shells, each sourced with ROS + the colcon ws)
```bash
# sim (headless; starts an Xvfb for camera rendering — see finding #3)
scripts/run.sh sim
# trainer (needs a torch+rclpy python; the depth_bench conda env has both)
python scripts/ppo_balance.py --env balance --run-dir $PWD/training_results/run1 ...
# real-Gazebo recorder alongside the trainer
scripts/run.sh record
```
The all‑in‑one resilient launcher used during development is `~/mybot_train_resilient.sh`
(Xvfb + world1 sim + recorder + trainer, auto‑restart on wedge). Results go to
`training_results/run1/` (metrics/, videos/, model_*.pth, train.log) on the **Windows** side.

## Environment specifics (verified)
- WSL2 Ubuntu‑22.04, ROS 2 Humble, Gazebo Classic 11. Torch+rclpy python =
  `~/anaconda3/envs/depth_bench/bin/python`. Isolation: `ROS_DOMAIN_ID=42`,
  `GAZEBO_MASTER_URI=http://localhost:11346` (never blanket‑kill gzserver — the user may
  run other sims).
- Edit on Windows, `rsync` into `~/ros2_ws/src` and `colcon build --symlink-install`.

## Hard‑won findings (KEEP THESE)
1. **Trust the eval, not raw `ep_len`.** `/gazebo/model_states` intermittently wedges to
   zeros on WSL, which faked long episodes. The env now reads tilt from the **IMU** and
   base velocity from **wheel odometry**. Always eval a fresh checkpoint.
2. **world1.world needs the balance physics + state plugin.** Copy balance.world's `<ode>`
   solver/constraints (esp. `contact_max_correcting_vel=1.0`) and the `gazebo_ros_state`
   plugin into any world used for training, or the robot's floor contacts explode
   (joint velocities → overflow → IMU garbage → the env declares a "wedge" and restarts
   forever). Set `real_time_update_rate=0` for faster‑than‑real‑time.
3. **Real Gazebo video works headless only under a virtual display.** gzserver renders a
   camera sensor only with an X display; WSLg `:0` isn't reliably present headless. Run
   gzserver under **Xvfb** (`run.sh sim` and the trainer do). `periodic_recorder.py` then
   records a real mp4 (`gazebo_ep<N>.mp4`) — the chosen "stC" framing (pose `0 4 3.4 0 0.6
   -1.5708`) is the robot in the foreground with the staircase behind it, in `world1`.
   Caveat: software‑GL (llvmpipe) rendering competes with the CPU‑bound trainer, so clips
   captured *during* training can be low‑fps / 0‑frame — record in a lighter‑load window.
4. **⚠️ LEG‑HOLDING IS THE OPEN BLOCKER.** The 8 leg joints are meant to be revolute +
   separately controlled (held at a stance for S1, RL‑driven for S2 walking). But
   **Gazebo Classic cannot hold these load‑bearing chained joints**: they buckle to a
   fixed collapsed equilibrium (~2.45 rad) the instant physics runs — *before* any
   controller activates — and then kinematically lock. Confirmed dead ends: `position`
   (SetPosition teleport), `position_pid` (PID→SetForce, gains 200–800), `effort`+env‑PD,
   `<dynamics damping/friction>` (10→8000 / 100), paused‑start (controllers won't activate
   paused), `reset_simulation`; there is no `set_model_configuration` service in ROS2
   gazebo_ros to straighten joints. The **only** working stance is **fixed legs**
   (`lock_legs:=true` → the 8 leg joints become `fixed`, dropped from ros2_control; the
   robot is then a rigid inverted pendulum and PPO gets a real balance signal). Toggle via
   `LOCK_LEGS` env / launch arg + the `lock_legs` xacro arg (mybot_geometry/mybot_balance).
   Proper revolute+actuated legs (walking) need a sim that holds load‑bearing joints
   (MuJoCo / Isaac / Gazebo Sim), not Gazebo Classic.
5. **world1 spawn race.** The large world1 loads slowly; the robot `spawn_entity` is delayed
   (`TimerAction`) to avoid "Spawn service failed", but it can still race intermittently —
   the resilient wrapper retries.

## Curriculum training (one continuum, ONE folder)
`scripts/train_curriculum.sh [start-stage]` runs the whole skill curriculum
autonomously: 2a balance-with-legs → 2b goal-nav → 3a wide-SQUAT-range re-balance → 3b
obstacles+ducking → 4 stairs/terrain. Stages ADVANCE automatically when learned
(`--advance-ep-rew/--advance-ep-len` rolling thresholds; `--total-timesteps` is only a
safety cap) and each warm-starts from the previous `model_best.pth` (weight surgery
when the obs grows). Portable: paths derive from the script location; set `PYTHON=`.

**ALL stages record into ONE run folder** (default `training_results/curriculum1`) —
the user explicitly wants this, never per-stage folders: one continuous per-episode
CSV, one continuous set of graph PNGs / `_overview.png` (series persisted in
`graphs/_series.json` and reloaded by the next stage), one TensorBoard record, and
episode/step counters that carry across stage boundaries (resumed from the CSV, the
source of truth). The run-dir ROOT holds the continuum `model_best/latest/final`
(the all-knowledge models); each stage's best is snapshotted into a NAMED SUBFOLDER
inside the run dir (`s2a_balance_legs/`, `s3b_obstacles_duck/`, ...). Ducking must be
learned as SQUATTING (pitch/knee joints, wide 0.7 rad range), never leg-splaying (hip
yaw/roll stay at 0.25 rad and fully posture-penalized) — verify with
`duck_eval.py --ckpt <run>/model_best.pth`.

**Hard-won finding #6 — leg-range widening breaks warm-starts.** The Stage-2 policy
saturates ~87% of its leg commands against the [-1,1] action clip, so when `leg_scale`
grows (0.25 → 0.7 rad, needed for ducking) NO weight rescale can preserve behavior
(clipping is nonlinear; a row-rescale un-saturates railed commands into 0.7 rad bends
→ instant collapse — measured, not theory). The fix is the same split-stage pattern
that cracked Stage 2: re-learn balance at the new range with the task stripped away
(`mujoco_obstacle_bal`, stage 3a), THEN add obstacles. When changing leg_scale between
stages, always pass `--load-leg-old-scale <old>` (rescales outputs + log-std for the
unsaturated regime) and budget a re-adaptation stage.

## STANDARD RESULTS LAYOUT — every trainer records EVERY run like this
The user considers this layout (as produced for `training_results/mujoco_run2`) the
standard. **Any new trainer / training stage MUST write the same structure** — never
just a bare model file + train.log:

```
<run_dir>/
├── model_best.(pth|pkl)        ← best policy so far (viewer hot-reloads this)
├── model_latest.(pth|pkl)      ← frequent resume copy
├── model_episode_<N>.(pth|pkl) ← numbered ckpt every --checkpoint-every EPISODES
├── model_final.(pth|pkl)       ← end of training
├── train.log
├── videos/                     ← clips (mujoco_view.py --record / recorders)
└── metrics/
    ├── episodes_<session>.csv  ← one row per episode: episode,score,steps,duration,
    │                             fell,mean_tilt_deg,max_tilt_deg,mean_effort,
    │                             global_step,timestamp   (metrics_logger.py)
    ├── summary_<session>.json  ← session_info + performance_metrics + recent
    ├── runs/<session>/         ← TensorBoard events (torch trainers only)
    └── graphs/                 ← LIVE PNGs: one per scalar + _overview.png,
                                  incl. episode__score/reward/steps/duration/
                                  mean_tilt_deg, charts__ep_rew/len_mean, sps,
                                  loss__policy/value/entropy/kl (tb_plot_writer.py)
```

Shared pieces (reuse, don't reimplement): `metrics_logger.py` (CSV+JSON),
`tb_plot_writer.py` — `TBPlotWriter` (TB events + PNGs, needs torch) for
`ppo_balance.py`/`ppo_parallel.py`, `PngPlotWriter` (PNGs only, torch-free) for
`mjx_ppo.py` in the `mybot_mjx` env (which has matplotlib but NO torch). MJX `.pkl`
checkpoints must be pickled as **plain lists** (numpy-version-portable — see
`mjx_pkl_fix.py` for the one-off converter for old files).

## Conventions
- No commit attribution (no Co‑Authored‑By/Claude/Anthropic).
- Don't touch `../collision_avoidance_system` (reference only).
- Keep scripts consolidated; don't scatter new files.
- Checkpoints: PyTorch trainers save `.pth`; the JAX/MJX trainer saves `.pkl`.
  `mujoco_view.py` auto-detects both and hot-reloads `model_best` while training.
