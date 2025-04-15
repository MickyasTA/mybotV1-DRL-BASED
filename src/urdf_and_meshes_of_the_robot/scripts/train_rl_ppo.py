#!/usr/bin/env python3
import gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback
import rclpy
import sys
import os
import time
import argparse

# Import our robot environment
from env import RobotEnv

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Train a PPO agent for the two-wheeled bipedal robot')
    parser.add_argument('--timesteps', type=int, default=100000, help='Total timesteps to train')
    parser.add_argument('--save-freq', type=int, default=10000, help='Frequency to save model checkpoints')
    parser.add_argument('--log-dir', type=str, default='./logs', help='Directory to save logs')
    parser.add_argument('--model-dir', type=str, default='./models', help='Directory to save models')
    parser.add_argument('--load-model', type=str, default=None, help='Path to load a pre-trained model')
    parser.add_argument('--check-env', action='store_true', help='Run environment check before training')
    args = parser.parse_args()
    
    # Create directories if they don't exist
    os.makedirs(args.log_dir, exist_ok=True)
    os.makedirs(args.model_dir, exist_ok=True)
    
    # Initialize ROS2
    rclpy.init()
    
    print("Creating robot environment...")
    
    # Create the environment
    env = RobotEnv()
    
    # Validate the environment if requested
    if args.check_env:
        print("Checking environment...")
        check_env(env)
        print("Environment check passed!")
    
    # Create checkpoint callback
    checkpoint_callback = CheckpointCallback(
        save_freq=args.save_freq,
        save_path=args.model_dir,
        name_prefix="bipedal_robot_model"
    )
    
    # Create or load the model
    if args.load_model:
        print(f"Loading pre-trained model from {args.load_model}")
        model = PPO.load(args.load_model, env=env)
    else:
        print("Creating new PPO model")
        model = PPO(
            "MlpPolicy", 
            env, 
            verbose=1,
            tensorboard_log=args.log_dir,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.0,
            vf_coef=0.5,
            max_grad_norm=0.5
        )
    
    # Train the model
    print(f"Starting training for {args.timesteps} timesteps...")
    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=checkpoint_callback
        )
        
        # Save the final model
        final_model_path = os.path.join(args.model_dir, "bipedal_robot_final_model")
        model.save(final_model_path)
        print(f"Training completed! Final model saved to {final_model_path}")
        
    except KeyboardInterrupt:
        print("Training interrupted by user")
        # Save the model on interrupt
        interrupt_model_path = os.path.join(args.model_dir, "bipedal_robot_interrupted_model")
        model.save(interrupt_model_path)
        print(f"Interrupted model saved to {interrupt_model_path}")
    
    # Close the environment
    env.close()
    
    print("Training script completed")

if __name__ == '__main__':
    main()