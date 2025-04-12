#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float64, Float32MultiArray
import numpy as np
import threading # used in the callback functions to avoid blocking the main thread( or the main loop)
import time

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Initialize joint positions dictionary
        self.joint_positions = {}
        self.joint_veleocities = {}

        # Initialize sensor data storage
        self.imu_data = None
        self.camera_data = None
        self.laser_data = None

        # Robot state tracking 
        self.robot_height = 0.0
        self.robot_orentation = [0.0, 0.0, 0.0, 0.0] # Quaternion orientation
        self.robot_linear_accel = [0.0, 0.0, 0.0] # Linear acceleration
        self.robot_angular_vel = [0.0, 0.0, 0.0] # Angular acceleration
        self.is_fallen = False

        # Lock for thread safety access to state variables
        self.state_lock = threading.Lock() # used to protect the shared state variables from concurrent access by multiple threads 

        # Subscribers to receive sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.camera_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/gazebo_ros_laser_controller/out',
            self.laser_callback,
            10
        )

        # Publisher to send commands to the wheels (for movement)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for joint control 
        # using effort controllers for the leg joints
        self.upper_left_pub = self.create_publisher(Float64, '/Upper_left_joint_effort_controller/command', 10)
        self.upper_right_pub = self.create_publisher(Float64, '/Upper_right_joint_effort_controller/command', 10)
        self.lower_left_pub = self.create_publisher(Float64, '/Lower_left_joint_effort_controller/command', 10)
        self.lower_right_pub = self.create_publisher(Float64, '/Lower_right_joint_effort_controller/command', 10)

        # Publisher for wheel joints (using position controllers)
        self.left_wheel_pub = self.create_publisher(Float64, '/Left_joint_position_controller/command', 10)
        self.right_wheel_pub = self.create_publisher(Float64, '/Right_joint_position_controller/command', 10)

        # Publisher to control specific joints
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_trajectory_controller/command', 10)

        # Timer to periodically publish commands for checking the robot state
        self.timer = self.create_timer(0.1, self.check_robot_state)  # Check every 0.1 seconds
        
        self.get_logger().info("Robot Control Node Initialized")

    # Callback for joint states (wheel and leg joints)
    def joint_state_callback(self, msg):
        with self.state_lock:
            self.get_logger().info(f"Joint States Received: {msg.name}")
            for i, name in enumerate(msg.name):
                position = msg.position[i] if i < len(msg.position) else 0.0
                velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
                # Store joint positions and velocities in a dictionary
                self.joint_positions[name] = {
                    'position': position,
                    'velocity': velocity
                }
                # Log the joint states ( reduced to debug level to avoide console spamming)
                self.get_logger().debug(f"{name} position: {position}, velocity: {velocity}")

                
    # Callback for IMU data
    def imu_callback(self, msg):
        with self.state_lock:
            self.imu_data = msg
            self.robot_orentation = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
            self.robot_linear_accel = [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
            self.robot_angular_vel = [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]
            # Check if the robot is fallen based on IMU data
            # A simple check : if the robot is tilted too much, consider it fallen
            # This can be improved with more sophisticated logic
            roll, pitch, yaw = self.euler_from_quaternion(self.robot_orentation)
            if abs(roll) > 0.5 or abs(pitch) > 0.5:  # (Radians) Adjust threshold as needed
                self.is_fallen = True
            else:
                self.is_fallen = False
            self.get_logger().debug(f"IMU Data: Orientation: {self.robot_orentation}, Linear Accel: {self.robot_linear_accel}, Angular Vel: {self.robot_angular_vel}")
            self.get_logger().debug(f"Robot is {'fallen' if self.is_fallen else 'upright'}")
    
    # Callback for camera data
    def camera_callback(self, msg):
        with self.state_lock:
            self.camera_data = msg
            self.get_logger().debug(f"Camera Data Received: {len(msg.data)} bytes")
            # Process camera data (e.g., convert to numpy array) on latter stage of the project
    
    # Callback for laser scan data
    def laser_callback(self, msg):
        with self.state_lock:
            self.laser_data = msg
            self.get_logger().debug(f"Laser Scan Data Received: {len(msg.ranges)} ranges")
       
    # Publish movement command to the robot (e.g., for wheels)
    def publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"Publishing CMD: linear_x={linear_x}, angular_z={angular_z}")

    # Publish joint commands (e.g., to control the legs or arms)
    def publish_joint_command(self, joint_name, value,
     command_type='position'):
        """
        Publish command to a specific joint
        
        Args:
            joint_name: Name of the joint to control
            value: Command value (position, velocity, or effort)
            command_type: Type of command ("position", "velocity", or "effort")
        """
        if command_type == 'effort':
            # Select the appropriate publisher based on joint name
            if joint_name == 'Upper_left_joint':
                self.upper_left_pub.publish(Float64(data=value))
            elif joint_name == 'Upper_right_joint':
                self.upper_right_pub.publish(Float64(data=value))
            elif joint_name == 'Lower_left_joint':
                self.lower_left_pub.publish(Float64(data=value))
            elif joint_name == 'Lower_right_joint':
                self.lower_right_pub.publish(Float64(data=value))
        else:
            # Use the joint trajectory controller for position/velocity commands
            joint_state = JointState()
            joint_state.name = [joint_name]

            if command_type == 'position':
                joint_state.position = [value]
                joint_state.velocity = [0.0]
                joint_state.effort = [0.0]
            elif command_type == 'velocity':
                joint_state.position = [0.0]
                joint_state.velocity = [value]
                joint_state.effort = [0.0]
            
            self.joint_cmd_pub.publish(joint_state)
        self.get_logger().info(f"Publishing Joint Command: {joint_name} to {command_type} {value}")

    # Publish commands to all joints at once
    def publish_all_joint_commands(self, positions):
        """
        Publish position commands to all joints at once
        
        Args:
            positions: Dictionary mapping joint names to position values
        """
        Joint_State = JointState()
        Joint_State.name = list(positions.keys())
        Joint_State.position = list(positions.values())
        Joint_State.velocity = [0.0] * len(positions)
        Joint_State.effort = [0.0] * len(positions)

        self.joint_cmd_pub.publish(Joint_State)
        self.get_logger().info(f"Publishing All Joint Commands: {positions}")

    # Get the current state of the robot (for reinforcement learning)
    def get_robot_state(self):
        """
        Get the current state of the robot for reinforcement learning
        
        Returns:
            state: Dictionary containing all relevant state information
        """
        with self.state_lock:
            state = {
                'joint_positions': self.joint_positions.copy(),
                'joint_velocities': self.joint_velocities.copy(),
                'orientation': self.robot_orentation.copy(),
                'linear_acceleration': self.robot_linear_accel.copy(),
                'angular_velocity': self.robot_angular_vel.copy(),
                'is_fallen': self.is_fallen,
                # 'camera_data': self.camera_data,
                # Add more state information as needed in link with the project
            }    
            # Add laser data if available
            if self.laser_data is not None:
                state['laser_ranges'] = list(self.laser_data.ranges)

            return state 

    # Check the robot state periodically
    def check_robot_state(self):
        """
        Check the robot state periodically and take action if needed
        """
        with self.state_lock:
            if self.is_fallen:
                self.get_logger().warn("Robot is fallen! Taking action...")
                # Here you could implement recovery behaviors or reset the simulation
                # Reset the simulation 
                
                self.reset_simulation()
                # For now, just log the state
                #self.get_logger().info("Robot is upright. Continuing operation...")

    # Helper function to convert quaternion to euler angles
    def euler_from_quaternion(self, quaternion):
        """
        Convert quaternion to euler angles (roll, pitch, yaw)
        
        Args:
            quaternion: List or tuple of 4 elements representing the quaternion (x, y, z, w)
        
        Returns:
            roll, pitch, yaw: Euler angles in radians
        """
        x, y, z, w = quaternion
        
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp) # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
        # Add more helper functions as needed for your project
        # For example, you might want to add functions for resetting the simulation,
        # checking battery status, etc.
        # You can also add functions for more complex control logic
        # based on the sensor data received.
        # For example, you might want to implement obstacle avoidance,
        # path planning, etc. based on the laser scan data.

        # Helper function to reset the simulation
    def reset_simulation(self):
        """
        Reset the simulation to a known state
        """
        # This is a placeholder function. You can implement the logic to reset the simulation
        # For example, you might want to send a reset command to the Gazebo simulation
        self.get_logger().info("Resetting simulation...")
        # Implement the reset logic here
        # For example, you might want to send a service call to reset the simulation
        # or set the robot position to a known state.
        pass
        # You can also add functions for more complex control logic


def main(args=None):
    rclpy.init(args=args)

    # Create the RobotControlNode
    robot_control_node = RobotControlNode()

    # Initalize the robot standing pose - adjust the robot based on your  robots configuration
    initial_pose = {
        'Upper_left_joint': 0.1, # pose in radians
        'Upper_right_joint': -0.1,
        'Lower_left_joint': -0.2,
        'Lower_right_joint': 0.0,
    }

    #Set the initial pose of the robot
    robot_control_node.publish_all_joint_commands(initial_pose)

    # wait a momment for the robot to settle(stabilize)
    time.sleep(1.0)
    
    # Publish a small movement command to the robot
    robot_control_node.publish_cmd(0.1, 0.0)  # Move forward slowly

    try:
        # Spin the node to keep receiving data
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown when done
        robot_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
# This script is used to control the robot model in Gazebo simulation
# It subscribes to various sensor data (joint states, IMU, camera, laser)
# and publishes commands to the robot joints and wheels
# The script also checks the robot state periodically and takes action if needed        """
# The script is designed to be used with ROS2 and Gazebo simulation
# The robot model is defined in a URDF file with meshes
# The script can be extended to include more complex control logic
# based on the sensor data received
# The script can also be used for reinforcement learning
# or other robotic applications
# The script is designed to be run as a ROS2 node
# The script can be run using the following command:
# ros2 run urdf_and_meshes_of_the_robot robot_nodes_control
# The script can also be launched using a launch file
# The launch file can be used to launch the Gazebo simulation
# and the robot control node together
# The launch file can be used to set parameters
# and configure the robot model
# The launch file can also be used to set the world file            