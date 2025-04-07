#!/usr/bin/env python3
import gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import rclpy
import sys
import os

# Add the package to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import our robot environment
from urdf_and_meshes_of_the_robot.env import RobotEnv

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create and check the environment
    env = RobotEnv()
    
    # Uncomment to validate the environment
    # check_env(env)
    
    # Create the model
    model = PPO("MlpPolicy", env, verbose=1)
    
    # Train the model
    model.learn(total_timesteps=10000)
    
    # Save the model
    model.save("robot_ppo_model")
    
    # Close the environment
    env.close()

if __name__ == '__main__':
    main()