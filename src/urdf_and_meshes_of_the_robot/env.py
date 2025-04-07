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

class RobotEnv(gym.Env):
    """
    OpenAI Gym Environment for Robot Control in ROS2/Gazebo
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
        # Action space: [linear_x, angular_z]
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),  # Min linear and angular velocities
            high=np.array([1.0, 1.0]),   # Max linear and angular velocities
            dtype=np.float32
        )
        
        # Observation space (customize based on your robot's sensors)
        # For simplicity, using LaserScan ranges + wheel positions
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(360 + 6,),  # 360 laser readings + 6 joint positions
            dtype=np.float32
        )
        
        # Store sensor data
        self.laser_data = None
        self.joint_positions = {}
        self.reset_requested = False
        
        # Setup publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
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
        
        # Wait until we get initial sensor data
        self._wait_for_sensor_data()

    def _ros_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
    def _wait_for_sensor_data(self):
        """Wait until we receive initial sensor data"""
        self.node.get_logger().info("Waiting for sensor data...")
        timeout = 30.0  # seconds
        start_time = time.time()
        
        while (self.laser_data is None or len(self.joint_positions) < 6) and time.time() - start_time < timeout:
            time.sleep(0.1)
            
        if self.laser_data is None or len(self.joint_positions) < 6:
            self.node.get_logger().error("Timed out waiting for sensor data!")
        else:
            self.node.get_logger().info("Received all required sensor data!")
            
    def _laser_callback(self, msg):
        """Store laser scan data"""
        self.laser_data = np.array(msg.ranges, dtype=np.float32)
        
    def _joint_state_callback(self, msg):
        """Store joint state data"""
        for i, name in enumerate(msg.name):
            if name in ['Upper_left_joint', 'Upper_right_joint', 'Lower_left_joint', 
                       'Lower_right_joint', 'Left_joint', 'Right_joint']:
                self.joint_positions[name] = msg.position[i]
    
    def _get_observation(self):
        """Combine sensor data into observation vector"""
        # Fill in missing laser readings with max range
        laser_obs = np.ones(360) * 10.0  # Default max range
        if self.laser_data is not None:
            actual_length = min(360, len(self.laser_data))
            laser_obs[:actual_length] = self.laser_data[:actual_length]
            
        # Get joint positions in a consistent order
        joint_obs = np.zeros(6)
        joint_names = ['Upper_left_joint', 'Upper_right_joint', 'Lower_left_joint', 
                      'Lower_right_joint', 'Left_joint', 'Right_joint']
        
        for i, name in enumerate(joint_names):
            if name in self.joint_positions:
                joint_obs[i] = self.joint_positions[name]
                
        # Combine observations
        return np.concatenate([laser_obs, joint_obs])
    
    def _calculate_reward(self):
        """Calculate reward based on laser scan data"""
        if self.laser_data is None:
            return -1.0  # Penalty for no sensor data
            
        # Simple reward: penalize getting too close to obstacles
        min_distance = np.min(self.laser_data)
        
        # Collision penalty
        if min_distance < 0.2:
            return -10.0
            
        # Distance-based reward component
        distance_reward = min_distance if min_distance < 2.0 else 2.0
        
        # Forward motion reward
        forward_reward = 0.1
        
        return distance_reward + forward_reward
    
    def step(self, action):
        """Take an action and return next state, reward, done, info"""
        # Apply action: publish velocity command
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        self.cmd_vel_pub.publish(cmd)
        
        # Wait for effects of action to be captured by sensors
        time.sleep(0.1)
        
        # Get observation
        observation = self._get_observation()
        
        # Calculate reward
        reward = self._calculate_reward()
        
        # Check if episode is done
        done = False
        if self.laser_data is not None:
            min_distance = np.min(self.laser_data)
            if min_distance < 0.15:  # Collision
                done = True
                
        info = {}
        
        return observation, reward, done, info
    
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
