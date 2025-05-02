#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
"""
conda activate ros2_rl_env
ros2 run urdf_and_meshes_of_the_robot robot_test.py
"""
class RobotTester(Node):
    def __init__(self):
        super().__init__('robot_tester')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.upper_left_pub = self.create_publisher(Float64, '/Upper_left_joint_position_controller/command', 10)
        self.upper_right_pub = self.create_publisher(Float64, '/Upper_right_joint_position_controller/command', 10)
        self.timer = self.create_timer(1.0, self.test_movement)
        self.get_logger().info("Robot tester initialized!")
        
    def test_movement(self):
        # Test cmd_vel
        self.get_logger().info("Testing forward movement...")
        cmd = Twist()
        cmd.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd)
        time.sleep(2.0)
        
        self.get_logger().info("Testing rotation...")
        cmd = Twist()
        cmd.angular.z = 0.5
        self.cmd_vel_pub.publish(cmd)
        time.sleep(2.0)
        
        # Test joint movement
        self.get_logger().info("Testing upper joints...")
        self.upper_left_pub.publish(Float64(data=0.5))
        self.upper_right_pub.publish(Float64(data=-0.5))
        time.sleep(2.0)
        
        self.upper_left_pub.publish(Float64(data=-0.5))
        self.upper_right_pub.publish(Float64(data=0.5))
        time.sleep(2.0)
        
        # Stop
        self.get_logger().info("Stopping...")
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main():
    rclpy.init()
    node = RobotTester()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()