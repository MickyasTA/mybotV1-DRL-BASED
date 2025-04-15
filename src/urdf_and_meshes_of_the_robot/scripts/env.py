#!/usr/bin/env python3
# Reinforcement Learning Environment using gazebo and ROS2 for robot control 
import rclpy
from rclpy.node import Node
import numpy as np
import time
import gym
from gym import spaces
from sensor_msgs.msg import JointState, Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import threading

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteEntity , SpawnEntity
import os
import math 
from ament_index_python.packages import get_package_share_directory

class RobotEnv(gym.Env):
    """
    OpenAI Gym Environment for Two-Wheeled Bipedal Robot Control in ROS2/Gazebo
    """
    
    def __init__(self):
        super(RobotEnv, self).__init__()
        
        # Initialize ROS2 node
        rclpy.init()
        self.node = Node('robot_env_node')
        
        # Run ROS2 on a separate thread
        self.ros_thread = threading.Thread(target=self._ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Define action and observation spaces
        # Action space: [upper_left_joint, upper_right_joint, lower_left_joint, lower_right_joint, left_wheel_vel, right_wheel_vel]
        self.action_space = spaces.Box(
            low=np.array([-0.5, -0.5, -0.5, -0.5, -1.0, -1.0]),  # Min joint positions/velocities
            high=np.array([0.5, 0.5, 0.5, 0.5, 1.0, 1.0]),       # Max joint positions/velocities
            dtype=np.float32
        )
        
        # Observation space (customize based on your robot's sensors)
        # [6 joint positions, 6 joint velocities, 3 IMU orientation, 3 IMU linear accel, 3 IMU angular vel, 360 laser readings]
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(6 + 6 + 3 + 3 + 3 + 360,),  # 360 laser readings + 6 joint positions
            dtype=np.float32
        )
        
        # Store sensor data
        self.joint_positions = {}
        self.joint_velocities = {}
        self.laser_data = None
        self.imu_orientation = None
        self.imu_linear_accel = None
        self.imu_angular_vel = None
        self.is_fallen = False
        self.reset_requested = False

        # Lock for thread safety access to state variables
        self.lock = threading.Lock()
        # Setup publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # Publishers for joint states
        self.upper_left_pub = self.node.create_publisher(Float64, '/Upper_left_joint_position_controller/command', 10)
        self.upper_right_pub = self.node.create_publisher(Float64, '/Upper_right_joint_position_controller/command', 10)
        self.lower_left_pub = self.node.create_publisher(Float64, '/Lower_left_joint_position_controller/command', 10)
        self.lower_right_pub = self.node.create_publisher(Float64, '/Lower_right_joint_position_controller/command', 10)
        self.left_wheel_pub = self.node.create_publisher(Float64, '/Left_joint_position_controller/command', 10)
        self.right_wheel_pub = self.node.create_publisher(Float64, '/Right_joint_position_controller/command', 10)
        
        # Joint state publisher for all joints at once
        self.joint_state_pub = self.node.create_publisher(JointState, '/joint_trajectory_controller/command', 10)

        # Subscribers for sensor data
        self.laser_sub = self.node.create_subscription(
            LaserScan,
            '/gazebo_ros_laser_controller/out',
            self._laser_callback,
            10
        )
        
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self.imu_sub = self.node.create_subscription(
            Image,
            '/imu_plugin/out',
            self._imu_callback,
            10
        )

        # subscriber for model state to get the robot position
        self.model_state_sub = self.node.create_subscription(
            ModelState,
            '/gazebo/model_states',
            self._model_state_callback,
            10
        )
        # create client for Gazebo service to spawn and delete entities
        self.delete_entity_client = self.node.create_client(DeleteEntity, '/gazebo/delete_entity')
        self.spawn_entity_client = self.node.create_client(SpawnEntity, '/gazebo/spawn_entity')

        # Robot state tracking 
        self.robot_position = [0.0, 0.0, 0.0]  # x, y, z
        self.robot_orientation_euler = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.start_position = [0.0, 0.0, 0.5]  # Initial spawn position
        self.episode_start_time = time.time()
        self.last_action_time = time.time()
        self.episode_steps = 0
        self.max_episode_steps = 1000  # Max steps per episode

        # Wait until we get initial sensor data
        self._wait_for_sensor_data()
        self.node.get_logger().info("Robot environment initialized!")

    def _ros_spin(self):
        """ spin ROS2 node in a separate thread"""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
    def _wait_for_sensor_data(self):
        """Wait until we receive initial sensor data"""
        self.node.get_logger().info("Waiting for sensor data...")
        timeout = 30.0  # seconds
        start_time = time.time()

        required_data = False
        
        while not required_data and time.time() - start_time < timeout:
            with self.lock:
                # check if we have and joint positions for 6 joints
                joint_names = ['Upper_left_joint', 'Upper_right_joint', 'Lower_left_joint',
                              'Lower_right_joint', 'Left_joint', 'Right_joint']
                joints_ready = all(name in self.joint_positions for name in joint_names)

                # check if we have laser data
                laser_ready = self.laser_data is not None and len(self.laser_data) > 0
                # check if we have IMU data
                imu_ready = (self.imu_orientation is not None and 
                            self.imu_linear_accel is not None and
                            self.imu_angular_vel is not None)
                
                required_data = joints_ready and laser_ready and imu_ready
            time.sleep(0.1)
        if not required_data:
            self.node.get_logger().error("Timed out waiting for sensor data!")
        else:
            self.node.get_logger().info("Received all required sensor data!")
            
    def _laser_callback(self, msg):
        """Store laser scan data"""
        self.laser_data = np.array(msg.ranges, dtype=np.float32)
        
    def _joint_state_callback(self, msg):
        """Store joint state data"""
        with self.lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.joint_positions[name] = msg.position[i]
                if i < len(msg.velocity):
                    self.joint_velocities[name] = msg.velocity[i]
        
    def _imu_callback(self, msg):
        """ store IMU data"""
        with self.lock:
            # store orentation as quaternion
            self.imu_orientation = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]

            # store linear acceleration
            self.imu_linear_accel = [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]

            # store angular velocity
            self.imu_angular_vel = [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]

            # check if robot is fallen based on orientation
            roll, pitch, yaw = self._quaternion_to_euler(self.imu_orientation)
            if abs(roll) > 0.7 or abs(pitch) > 0.7: # radians
                self.is_fallen = True
            else:
                self.is_fallen = False
    def _model_state_callback(self, msg):
        """ store robot position and orientation from gazebo model state"""
        try:
            # find the index of the robot in the model state message
            if 'urdf_and_meshes_of_the_robot' in msg.name:
                index = msg.name.index('urdf_and_meshes_of_the_robot')

                # store position 
                self.robot_position = [
                    msg.pose[index].position.x,
                    msg.pose[index].position.y,
                    msg.pose[index].position.z
                ]

                # store orientation
                quat = [
                    msg.pose[index].orientation.x,
                    msg.pose[index].orientation.y,
                    msg.pose[index].orientation.z,
                    msg.pose[index].orientation.w
                ]
                self.robot_orientation_euler = self._quaternion_to_euler(quat)
        except (ValueError , IndexError):
            self.node.get_logger().error("Robot not found in model state message!")        
    def _get_observation(self):
        """Combine sensor data into observation vector"""
        # Fill in missing laser readings with max range
        with self.lock:
            # Get joint positions in a constrent order
            joint_pos = np.zeros(6)
            joint_vel = np.zeros(6)
            joint_names = ['Upper_left_joint', 'Upper_right_joint', 'Lower_left_joint',
                          'Lower_right_joint', 'Left_joint', 'Right_joint']
            for i, name in enumerate(joint_names):
                if name in self.joint_positions:
                    joint_pos[i] = self.joint_positions[name]
                if name in self.joint_velocities:
                    joint_vel[i] = self.joint_velocities[name]
            # Get IMU data
            if self.imu_orientation is not None:
                roll, pitch, yaw = self._quaternion_to_euler(self.imu_orientation)
                imu_orient = np.array([roll, pitch, yaw])
            else:
                imu_orient = np.zeros(3)
            if self.imu_linear_accel is not None:
                imu_accel = np.array(self.imu_linear_accel)
            else:
                imu_accel = np.zeros(3)
            if self.imu_angular_vel is not None:
                imu_ang_vel = np.array(self.imu_angular_vel)
            else:
                imu_ang_vel = np.zeros(3)

            # Get laser data
            laser_obs = np.zeros(360) *10.0 # default laser range max range
            if self.laser_data is not None:
                ranges = np.array(self.laser_data.ranges)
                # Clip to max range and handle inf/nan
                ranges = np.clip(ranges, 0.0, 10.0)
                ranges[~np.isfinite(ranges)] = 10.0  # Set inf/nan to max range

                # Downsample or upsample to 360 readings if needed
                if len(ranges) != 360:
                    indices = np.linspace(0, len(ranges) - 1, 360, dtype = int)
                    laser_obs = ranges[indices]
                else:
                    laser_obs = ranges

            # combine all observations
            observation = np.concatenate([
                joint_pos,
                joint_vel,
                imu_orient,
                imu_accel,
                imu_ang_vel,
                laser_obs
            ])

            return observation
    

    def _calculate_reward(self):
        """Calculate reward based on robot state"""
        reward = 0.0
        
        # Penalty for falling
        if self.is_fallen:
            return -100.0
        
        # Calculate distance traveled since last action
        current_pos = np.array(self.robot_position[:2])  # x, y position
        last_pos = np.array(self.last_position[:2])
        distance = np.linalg.norm(current_pos - last_pos)
        self.total_distance_traveled += distance
        self.last_position = self.robot_position.copy()
        
        # Reward for moving forward (in x direction)
        forward_reward = distance * 10.0
        reward += forward_reward
        
        # Penalty for tilting too much (but not enough to fall)
        if self.imu_orientation is not None:
            roll, pitch, _ = self._euler_from_quaternion(self.imu_orientation)
            tilt_penalty = -5.0 * (abs(roll) + abs(pitch))
            reward += tilt_penalty
        
        # Penalty for getting too close to obstacles
        if self.laser_data is not None:
            min_distance = min(self.laser_data.ranges)
            if min_distance < 0.3:
                obstacle_penalty = -10.0 * (0.3 - min_distance)
                reward += obstacle_penalty
        
        # Small penalty for using energy (encourage efficient movement)
        energy_penalty = -0.1
        reward += energy_penalty
        
        # Bonus for staying upright
        upright_bonus = 1.0
        reward += upright_bonus
        
        return reward

    def _take_action(self, action):
        """Apply the action to the robot"""
        # scale action to appropriate range
        upper_left = float(action[0])
        upper_right = float(action[1])
        lower_left = float(action[2])
        lower_right = float(action[3])
        left_wheel = float(action[4])
        right_wheel = float(action[5])

        # publish joint commands
        self._publish_joint_command("Upper_left_joint", upper_left, "effort")
        self._publish_joint_command("Upper_right_joint", upper_right, "effort")

        self._publish_joint_command("Lower_left_joint", lower_left, "effort")
        self._publish_joint_command("Lower_right_joint", lower_right, "effort")

        self._publish_joint_command("Left_joint", left_wheel, "velocity")
        self._publish_joint_command("Right_joint", right_wheel, "velocity")

        # update last action time
        self.last_action_time = time.time()


    def _publish_joint_command(self, joint_name, value, command_type="position"):
        """
        Publish command to a specific joint
        
        Args:
            joint_name: Name of the joint to control
            value: Command value (position, velocity, or effort)
            command_type: Type of command ("position", "velocity", or "effort")
        """
        if command_type == "effort":
            # Select the appropriate publisher based on joint name
            if joint_name == "Upper_left_joint":
                self.upper_left_pub.publish(Float64(data=value))
            elif joint_name == "Upper_right_joint":
                self.upper_right_pub.publish(Float64(data=value))
            elif joint_name == "Lower_left_joint":
                self.lower_left_pub.publish(Float64(data=value))
            elif joint_name == "Lower_right_joint":
                self.lower_right_pub.publish(Float64(data=value))
            elif joint_name == "Left_joint":
                self.left_wheel_pub.publish(Float64(data=value))
            elif joint_name == "Right_joint":
                self.right_wheel_pub.publish(Float64(data=value))
        else:
            # Use the joint trajectory controller for position/velocity control
            joint_state = JointState()
            joint_state.name = [joint_name]
            
            if command_type == "position":
                joint_state.position = [value]
                joint_state.velocity = [0.0]
                joint_state.effort = [0.0]
            elif command_type == "velocity":
                joint_state.position = [0.0]
                joint_state.velocity = [value]
                joint_state.effort = [0.0]
            else:
                raise ValueError("Invalid command type. Use 'position', 'velocity', or 'effort'.")
            self.joint_state_pub.publish(joint_state)

    # Convert quaternion to Euler angles
    def _quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        x, y, z, w = quat
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    # Convert Euler angles to quaternion
    def _euler_from_quaternion(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        x, y, z, w = quat
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    # Calculate distance and angle between two points
    def _calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two points"""
        return np.linalg.norm(np.array(pos1) - np.array(pos2))

    # Calculate angle between two points
    def _calculate_angle(self, pos1, pos2):
        """Calculate angle between two points"""
        delta_x = pos2[0] - pos1[0]
        delta_y = pos2[1] - pos1[1]
        return math.atan2(delta_y, delta_x)
    
    # def step(self, action):
    #     """Take an action and return next state, reward, done, info"""
    #     # Apply action: publish velocity command
    #     cmd = Twist()
    #     cmd.linear.x = float(action[0])
    #     cmd.angular.z = float(action[1])
    #     self.cmd_vel_pub.publish(cmd)
        
    #     # Wait for effects of action to be captured by sensors
    #     time.sleep(0.1)
        
    #     # Get observation
    #     observation = self._get_observation()
        
    #     # Calculate reward
    #     reward = self._calculate_reward()
        
    #     # Check if episode is done
    #     done = False
    #     if self.laser_data is not None:
    #         min_distance = np.min(self.laser_data)
    #         if min_distance < 0.15:  # Collision
    #             done = True
                
    #     info = {}
        
    #     return observation, reward, done, info
    
    def reset(self):
        """Reset the environment"""
        # Reset robot position (in a real implementation, you would call a service
        # to reset the simulation or teleport the robot)
        self.node.get_logger().info("Resetting environment...")
        
        # Request reset (in a real implementation)
        self.reset_requested = True
        
        # Wait for reset to complete and sensors to update
        time.sleep(1.0)
        
        # Clear the reset request flag
        self.reset_requested = False
        
        # Return initial observation
        return self._get_observation()
    
    def render(self, mode='human'):
        """
        Render the environment
        In ROS2/Gazebo, rendering is already handled by Gazebo
        """
        pass
    
    def close(self):
        """Clean up resources"""
        self.node.get_logger().info("Shutting down ROS node...")
        self.node.destroy_node()
        rclpy.shutdown()
