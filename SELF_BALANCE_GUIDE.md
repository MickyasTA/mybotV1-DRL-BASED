# Self-Balance Milestone — Setup & Run Guide

Milestone 1: teach the two-wheeled bipedal robot to **self-balance in place** with a
**from-scratch PPO** (no Stable-Baselines3), using **ros2_control** in Gazebo Classic.

This guide covers the new files, one-time setup, how to train, and how to verify/tune.

---

## What was added / changed

**New**
- `urdf/mybot_geometry.urdf.xacro` — robot links + joints (reused; wheel/leg limits widened). **Each hip is now 3-DOF** (`hip_yaw_*` + `hip_roll_*` + existing `Upper_*_joint` as pitch), via small primitive connector links — no new STL meshes needed.
- `urdf/mybot_balance.urdf.xacro` — training description: IMU + **ros2_control** (wheels=effort; 6 hips=position; 2 knees=position) + `gazebo_ros2_control`. Drops camera/lidar/diff_drive for this milestone.
- `config/controllers.yaml` — `joint_state_broadcaster`, `wheel_effort_controller` (2), `hip_position_controller` (6, policy), `knee_position_controller` (2, held).
- `worlds/balance.world` — flat world, `real_time_update_rate=0` (fast), `gazebo_ros_state` plugin for deterministic resets.
- `launch/balance.launch.py` — headless-capable bring-up + controller spawners.
- `scripts/balance_env.py` — Gymnasium env. **8-D action** = `[2 wheel torques, 6 hip angle targets]` (symmetric `[-1,1]`); proprioceptive obs (IMU + wheel/hip pos+vel + base vel + prev action, history-stacked); services-based reset; `/clock`-paced step; dense reward.
- `scripts/ppo_balance.py` — **from-scratch PPO** (ActorCritic, GAE, clipped surrogate, obs **and** reward normalization). Has a `--selftest`-style `--env Pendulum-v1` mode.

**Fixed**
- `CMakeLists.txt` — removed a buggy resource-marker block that broke `--symlink-install`; installs the new scripts.
- `package.xml` — added `xacro`, `controller_manager`, `ros2_control`, `ros2_controllers`, `gazebo_ros2_control`.
- `.gitignore` — colcon artifacts; removed the committed 3.6 MB `export.log`.

The original `scripts/env.py`, `train_rl_ppo.py`, `robot_nodes_control.py` are kept for reference but are **not** used by this milestone.

---

## One-time setup (WSL Ubuntu 22.04)

**1. Install the ros2_control runtime packages** (currently missing on this machine):
```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control ros-humble-xacro
```

**2. Create the conda env** (has torch+CUDA+gymnasium; `ros2_rl_env` does not exist yet):
```bash
conda env create -f /mnt/c/Users/mtasfaw/Documents/personal/mybotV1-DRL-BASED/ros2_rl_env.yml
```

**3. Sync the package into the ROS2 workspace and build:**
```bash
mkdir -p ~/ros2_ws/src
rsync -a --delete --exclude build --exclude install --exclude log \
  /mnt/c/Users/mtasfaw/Documents/personal/mybotV1-DRL-BASED/src/urdf_and_meshes_of_the_robot/ \
  ~/ros2_ws/src/urdf_and_meshes_of_the_robot/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select urdf_and_meshes_of_the_robot --symlink-install
source install/setup.bash
```
> Re-run the `rsync` + `colcon build` after each round of Windows-side edits.
> (The `~/ros2_ws` package already built successfully during setup; only the apt
> packages above are still required before launching.)

---

## Train

**Terminal 1 — bring up Gazebo + robot + controllers (headless):**
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch urdf_and_meshes_of_the_robot balance.launch.py          # add gui:=true to watch
# sanity:
ros2 control list_controllers      # expect 3 controllers 'active'
ros2 service list | grep -E 'set_entity_state|pause_physics'
```

**Terminal 2 — run the from-scratch PPO (conda env with torch):**
```bash
conda activate ros2_rl_env
cd ~/ros2_ws/src/urdf_and_meshes_of_the_robot/scripts
python ppo_balance.py --env balance --total-timesteps 2000000
# monitor:  tensorboard --logdir ./logs     (and ./logs/progress.csv)
```

Watch **`ep_len_mean` grow** (robot stays up longer) and **`ep_rew_mean` rise**.
Replay a trained policy with the GUI:
```bash
# (terminal 1) ros2 launch ... balance.launch.py gui:=true
python ppo_balance.py --env balance --play --load ./models/best.pt
```

---

## Verify the algorithm without the robot (already passed here)

```bash
python ppo_balance.py --env Pendulum-v1 --total-timesteps 200000 --device cpu
```
A correct PPO climbs from **~ -1200 (random) to ~ -180** — confirmed during development.

Env contract smoke test (needs Gazebo running in terminal 1):
```bash
python balance_env.py    # prints obs/action dims, steps randomly, shows reset-after-fall
```

---

## Tuning knobs (expect an empirical pass — see plan caveats)

In `balance_env.py` (`BalanceEnv.__init__`):
- **`spawn_z`** — set so the wheels rest on the ground at reset (launch arg `spawn_z` too).
- **`q_hip_nom`** (6) / **`q_knee_nom`** (2) — nominal stance; must place the body CoM over the wheel axle. Start at zeros, adjust while watching `gui:=true`.
- **`hip_action_scale`** — rad per unit hip action about `q_hip_nom` (default 0.5). Lower it (e.g. 0.2) if the actuated hips make early balance training unstable.
- The hips are **actuated by the policy** (8-D action). To temporarily freeze them and train wheels-only, set `hip_action_scale=0.0` (they hold `q_hip_nom`).
- **Pitch axis** — the reward penalizes both `roll`+`pitch`, so axis identification isn't required, but confirm which IMU axis is the unstable tip direction by printing IMU during a manual push.
- **Reward weights** `w_alive,w_tilt,w_tilt_rate,w_effort,w_drift,fall_penalty` and **`fall_tilt`** — paper coefficients were refuted by the research; tune from the defaults.
- **`max_wheel_effort`** — torque scale for `action∈[-1,1]` (URDF limit is 20 Nm).
- **`use_pause=False`** — fallback if pause/unpause-per-step is flaky on Humble (runs continuously; relies on the observation history for latency).

In `ppo_balance.py`: standard PPO args (`--gamma` 0.99→0.999 for longer horizon, `--lr`, `--clip`, `--ent-coef`, `--n-steps`, etc.).

---

## Roadmap (after balancing works)
S2 velocity tracking → S3 full wheel-legged (6-D action) → S4 sim-to-real (domain randomization, latency) → S5 navigation (re-enable lidar/camera + diff-drive). See the approved plan for details.
