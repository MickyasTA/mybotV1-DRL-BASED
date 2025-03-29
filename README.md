# DRL Based Wheeled Bipedal Robot in ROS 2 and Gazebo

This repository contains the URDF, meshes, and launch files for simulating a **Wheeled Bipedal Robot** in ROS 2 using Gazebo and RViz2. The final goal is to control and train the robot using Reinforcement Learning (RL) techniques.

## 🧠 Project Goals

1. Visualize the robot model in RViz2.
2. Simulate the robot in Gazebo with realistic dynamics.
 3. Integrate sensors (IMU, encoders) and actuators (motors). -(on-progress)
4. Collect sensor data through ROS 2 topics.
5. Train the robot using Reinforcement Learning (future steps).

---

## 🚀 Installation and Setup

### 1. Clone the Repository
```bash
git clone https://github.com/MickyasTA/mybotV1-DRL-BASED.git
cd ~/mybotV1-DRL-BASED
```

### 2. Build the Workspace
```bash
colcon build --symlink-install
```

### 3. Source the Workspace
```bash
source install/setup.bash
```

### 4. 🏁 Run the Simulation
To visualize the robot in Gazebo and RViz2:
```bash
ros2 launch urdf_and_meshes_of_the_robot display.launch.py
```

