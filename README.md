# Two-Wheeled Bipedal Robot Reinforcement Learning Environment

This package provides a complete reinforcement learning environment for a two-wheeled bipedal robot in Gazebo with ROS2. The robot can learn to balance and navigate using reinforcement learning algorithms.

## Package Structure

```
urdf_and_meshes_of_the_robot/
├── config/                  # Configuration files
│   └── urdf.rviz            # RViz configuration
├── launch/                  # Launch files
│   ├── display.launch.py    # Launch file for visualization
│   ├── gazebo.launch.py     # Launch file for Gazebo simulation
│   └── rl_environment.launch.py # Launch file for RL environment
├── meshes/                  # 3D model meshes
├── scripts/                 # Python scripts
│   ├── env.py               # RL environment implementation
│   ├── robot_nodes_control.py # Robot control node
│   └── train_rl_ppo.py      # PPO training script
├── urdf/                    # URDF model files
│   └── urdf_and_meshes_of_the_robot.urdf # Robot URDF model
└── resource/                # Package resources
```

## Prerequisites

- ROS2 (tested with Humble)
- Gazebo
- Python 3.8+
- OpenAI Gym
- Stable Baselines3

## Installation

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> urdf_and_meshes_of_the_robot
   ```

2. Install dependencies:
   ```bash
   sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-joint-state-publisher-gui
   pip install gym stable-baselines3 gymnasium
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage

### Visualize the Robot in RViz

```bash
ros2 launch urdf_and_meshes_of_the_robot display.launch.py
```

### Launch the Robot in Gazebo

```bash
ros2 launch urdf_and_meshes_of_the_robot gazebo.launch.py
```

### Launch the Complete RL Environment

```bash
ros2 launch urdf_and_meshes_of_the_robot rl_environment.launch.py
```

### Train the RL Agent

```bash
ros2 launch urdf_and_meshes_of_the_robot rl_environment.launch.py train_rl:=true
```

Alternatively, you can run the training script directly:

```bash
ros2 run urdf_and_meshes_of_the_robot train_rl_ppo.py --timesteps 100000 --save-freq 10000
```

## Reinforcement Learning Environment

The reinforcement learning environment is implemented using the OpenAI Gym interface. The environment provides:

- **Observation Space**: Joint positions, velocities, IMU data, and laser scan readings
- **Action Space**: Control commands for leg joints and wheel velocities
- **Reward Function**: Rewards for maintaining balance, moving forward, and avoiding obstacles

## Robot Control

The robot control node handles sensor data processing and actuator control. It provides interfaces for:

- Reading joint states, IMU data, camera images, and laser scans
- Publishing control commands to joints and wheels
- Monitoring robot state for balance and collision detection

## Customization

You can customize the reinforcement learning parameters in `train_rl_ppo.py` and the environment settings in `env.py`. The robot model can be modified in the URDF file.

## Troubleshooting

- If the robot falls immediately, check the initial joint positions in `robot_nodes_control.py`
- If sensors are not publishing data, check the topic names in `env.py`
- For visualization issues, check the RViz configuration in `config/urdf.rviz`

## License

This package is licensed under the BSD License.