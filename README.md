# mybotV1 — Wheel‑Legged Bipedal Robot, Reinforcement Learning

A **two‑wheeled bipedal (wheel‑legged) robot** trained with a **from‑scratch PPO**
(no Stable‑Baselines3 / no RL library) in **ROS 2 Humble + Gazebo Classic**.

The robot has **two independent legs** (each a 3‑DOF hip + knee) ending in driven
**wheels**. The staged goal:

1. **Stage 1 — balance in place** on the wheels (this repo's current milestone).
2. **Stage 2+ — walk / climb stairs** with the two legs *while* the wheels turn
   (the legs are free, independently‑actuated joints so a later policy can move them).

> **Honest status (read this):** the learning pipeline is real and the training
> signal is now trustworthy (see *Key findings* below). With the corrected signal the
> robot **genuinely learns** (eval episode‑length climbs), but **robust balancing is
> still a tuning work‑in‑progress**, and **stably holding the freed legs** under
> gravity in Gazebo Classic is also WIP. This README is honest about what works and
> what doesn't so you can reproduce and continue it.

---

## 1. What's in here

```
mybotV1-DRL-BASED/
├── README.md                    ← this file
├── ros2_rl_env.yml              ← conda env (Python 3.10 + torch + gymnasium…)
├── requirements.txt             ← pip deps (alternative to the conda env)
├── SELF_BALANCE_GUIDE.md        ← deeper notes on the self‑balance design
└── src/urdf_and_meshes_of_the_robot/      ← the ROS 2 package
    ├── package.xml  CMakeLists.txt
    ├── urdf/        ← robot xacro/urdf (mybot_geometry, mybot_balance)
    ├── meshes/      ← STL meshes
    ├── worlds/      ← balance.world (flat training world)
    ├── config/      ← controllers.yaml, gazebo_params.yaml
    ├── launch/      ← balance.launch.py (headless sim + ros2_control)
    └── scripts/
        ├── balance_env.py        ← Gymnasium env (IMU tilt + wheel odometry, ros2_control)
        ├── ppo_balance.py        ← from‑scratch PPO trainer (+ eval / play modes)
        ├── record_episode.py     ← FK MP4 render of an episode (matplotlib)
        ├── periodic_recorder.py  ← REAL headless‑Gazebo video via a spawned camera
        ├── tb_plot_writer.py     ← TensorBoard + live PNG graphs
        ├── metrics_logger.py     ← per‑episode CSV + summary JSON
        ├── plot_metrics.py       ← overview‑grid plot
        ├── camera_recorder.py    ← (optional) onboard‑camera MP4 helper
        └── env.py / train_rl_ppo.py / robot_nodes_control.py / robot_test.py
                                   ← original baseline (kept for reference; NOT used)
```

The **active** stack is `balance_env.py` + `ppo_balance.py` + the launch/world/config.
The `env.py` / `train_rl_ppo.py` files are the original SB3‑based baseline, kept only
for reference.

---

## 2. Prerequisites

- **Ubuntu 22.04** (native or **WSL2**); NVIDIA GPU optional (the policy is a tiny MLP,
  so CPU is fine — and frees the GPU for other work).
- **ROS 2 Humble** + **Gazebo Classic 11** + the ros2_control stack:
  ```bash
  sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs \
       ros-humble-gazebo-ros2-control ros-humble-ros2-control \
       ros-humble-ros2-controllers ros-humble-xacro ros-humble-cv-bridge
  ```
- **Python 3.10** with `torch`, `gymnasium`, `numpy`, `matplotlib`, `opencv-python`
  (see step 3).

---

## 3. Install & build

```bash
# 1. Clone INTO a colcon workspace's src/
mkdir -p ~/ros2_ws/src
git clone <this-repo-url> ~/ros2_ws/src/mybotV1-DRL-BASED
# the ROS 2 package is at  src/mybotV1-DRL-BASED/src/urdf_and_meshes_of_the_robot

# 2. Python deps — either the conda env …
conda env create -f ~/ros2_ws/src/mybotV1-DRL-BASED/ros2_rl_env.yml && conda activate ros2_rl_env
#    … or plain pip into any Python 3.10:
pip install -r ~/ros2_ws/src/mybotV1-DRL-BASED/requirements.txt

# 3. Build the ROS 2 package
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

> Editing on Windows, running in WSL? `rsync` the package into the workspace `src/` and
> re‑run `colcon build` (symlink‑install picks up script edits without a rebuild).

---

## 4. Run

Open **two shells** (each: `source /opt/ros/humble/setup.bash` + `source ~/ros2_ws/install/setup.bash`).

**Shell A — the simulator** (headless; Gazebo Classic GUI can't render this robot on
WSL — see *Key findings*):
```bash
ros2 launch urdf_and_meshes_of_the_robot balance.launch.py gui:=false
```
Brings up gzserver + `robot_state_publisher` + spawns the robot + activates the
controllers (`joint_state_broadcaster`, `wheel_effort_controller`, `leg_position_controller`).

**Shell B — the trainer** (from‑scratch PPO):
```bash
export PYTHONPATH=~/ros2_ws/src/mybotV1-DRL-BASED/src/urdf_and_meshes_of_the_robot/scripts:$PYTHONPATH
python ppo_balance.py --env balance --device cpu \
    --max-episodes 5000 --record-first 100 --record-every 400 \
    --run-dir $PWD/training_results/run1 --session-id $(date +%Y%m%d_%H%M%S)
```

Validate the PPO itself (no robot needed — sanity check the algorithm):
```bash
python ppo_balance.py --env Pendulum-v1 --total-timesteps 100000   # return should rise toward ~-200
```

**Optional — real Gazebo video while training** (headless, no gzclient): run alongside
the trainer; every 400 episodes it spawns a temporary camera into the live world,
records ~40 episodes to an mp4, then deletes the camera:
```bash
REC_DIR=$PWD/training_results/run1/videos python periodic_recorder.py
```

---

## 5. Results layout

Each run writes a self‑describing folder:
```
training_results/run1/
├── metrics/
│   ├── episodes_<session>.csv     ← per‑episode: score, steps, duration, fell, tilt, effort
│   ├── summary_<session>.json     ← session + performance summary
│   ├── runs/<session>/            ← TensorBoard event files
│   └── graphs/  <metric>.png + _overview.png   ← live PNG graphs (raw + EMA)
├── videos/  ep<N>.mp4             ← episode recordings (FK render, or real‑Gazebo if periodic_recorder)
├── model_episode_<N>.pth, model_best.pth, model_latest.pth, model_final.pth
└── train.log
```
**Judge balancing by the milestone EVAL length** (deterministic 50‑episode eval) in
`train.log` / `metrics/eval.csv`, **not** the raw training `ep_len` (see finding #1).

---

## 6. Key findings & gotchas (hard‑won — please keep these)

1. **The training signal was corrupted by `/gazebo/model_states`.** That plugin
   intermittently wedges to all‑zeros on WSL; reading tilt from it made fallen
   episodes look long, so the policy was rewarded for *fake* balancing. **Fix:** the
   env now reads tilt/rate from the **IMU** (`/imu`, with its 90° mount offset undone)
   and base velocity from **wheel odometry** — independent of that plugin.
   *Always eval a fresh checkpoint to judge balancing; the raw `ep_len` can lie.*
2. **Reward shaping matters.** Balancing *requires* driving the wheels, so the drift /
   tilt‑rate penalties were lowered (they were punishing the solution).
3. **Gazebo Classic GUI can't render this robot on WSL** — `gzclient` (and onboard
   camera sensors) crash in OGRE (`rendering::Camera px != 0`) on the STL meshes.
   `record_episode.py` therefore renders the **true robot pose** with matplotlib (FK);
   `periodic_recorder.py` records via a **spawned camera** entity (needs a display /
   Xvfb for gzserver's camera). RViz *does* render the robot live.
4. **Freed legs are WIP.** The 8 leg joints are `revolute` (independent), held by a
   `JointGroupPositionController`, but Gazebo Classic doesn't hold load‑bearing
   position‑controlled joints well against gravity — stable holding/actuation is the
   next task (effort‑PID, or a stronger simulator).
5. **Faster‑than‑real‑time** training needs `/clock` ≥ control rate: see
   `config/gazebo_params.yaml` (`publish_rate`) — the gazebo default 10 Hz made the
   effective control rate 10 Hz.

---

## 7. Roadmap

- **S1 (now):** robustly balance in place on the wheels (reward/exploration tuning).
- **S2:** stably hold + then actuate the legs (effort‑PID); add leg targets to the action.
- **S3:** walk / climb stairs with the legs while the wheels roll.
- **Sim note:** the leg‑actuation + live‑rendering limits above are Gazebo‑Classic‑on‑WSL
  issues; native‑Linux Gazebo or Isaac/MuJoCo would remove them for the full vision.
