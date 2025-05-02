#!/usr/bin/env python3
"""
conda activate ros2_rl_env

cd logs
tensorboard --logdir=./PPO_4
"""
import gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
import rclpy
import sys
import os
import time
import argparse
from rclpy.utilities import remove_ros_args
import torch

# Import our robot environment
from env import RobotEnv

# Enhanced logging callback for better visibility into training progress
class EnhancedLoggingCallback(BaseCallback):
    def __init__(self, save_freq, save_path, name_prefix="model", verbose=1):
        super().__init__(verbose)
        self.save_freq = save_freq
        self.save_path = save_path
        self.name_prefix = name_prefix
        self.episode_rewards = []
        self.episode_lengths = []
        self.current_episode_reward = 0
        self.n_falls = 0
        self.best_mean_reward = -float('inf')
        
    def _on_step(self):
        # Save model periodically
        if self.n_calls % self.save_freq == 0:
            path = os.path.join(self.save_path, f"{self.name_prefix}_{self.n_calls}_steps")
            self.model.save(path)
            if self.verbose > 0:
                print(f"Saving model checkpoint to {path}")
        
        # Get the most recent episode info
        if len(self.model.ep_info_buffer) > 0 and len(self.model.ep_info_buffer[-1]) > 0:
            info = self.model.ep_info_buffer[-1]
            rewards = self.locals.get("rewards")[0]
            
            # Accumulate reward
            self.current_episode_reward += rewards
            
            # Check if episode is done
            dones = self.locals.get("dones")
            if dones and dones[0]:
                # Episode finished
                ep_reward = info.get('r', self.current_episode_reward)
                ep_length = info.get('l', 0)
                self.episode_rewards.append(ep_reward)
                self.episode_lengths.append(ep_length)
                
                # Check if robot fell (from additional info)
                additional_info = self.locals.get("infos")[0]
                if additional_info.get("is_fallen", False):
                    self.n_falls += 1
                
                # Calculate statistics
                mean_reward = np.mean(self.episode_rewards[-10:]) if len(self.episode_rewards) >= 10 else np.mean(self.episode_rewards)
                mean_length = np.mean(self.episode_lengths[-10:]) if len(self.episode_lengths) >= 10 else np.mean(self.episode_lengths)
                
                # Save best model
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    path = os.path.join(self.save_path, f"{self.name_prefix}_best_model")
                    self.model.save(path)
                    if self.verbose > 0:
                        print(f"Saving new best model to {path}")
                
                # Print detailed info
                print(f"\n{'='*50}")
                print(f"Episode: {len(self.episode_rewards)}")
                print(f"Reward: {ep_reward:.2f} (Avg last 10: {mean_reward:.2f})")
                print(f"Length: {ep_length} steps (Avg last 10: {mean_length:.2f})")
                print(f"Total falls: {self.n_falls}")
                
                # Print additional info if available
                if "total_distance_traveled" in additional_info:
                    print(f"Distance traveled: {additional_info['total_distance_traveled']:.2f} m")
                if "robot_position" in additional_info:
                    pos = additional_info["robot_position"]
                    print(f"Final position: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")
                
                print(f"{'='*50}")
                
                # Reset episode reward
                self.current_episode_reward = 0
        
        return True

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Train a PPO agent for the two-wheeled bipedal robot')
    parser.add_argument('--timesteps', type=int, default=100000, help='Total timesteps to train')
    parser.add_argument('--save-freq', type=int, default=10000, help='Frequency to save model checkpoints')
    parser.add_argument('--log-dir', type=str, default='./logs', help='Directory to save logs')
    parser.add_argument('--model-dir', type=str, default='./models', help='Directory to save models')
    parser.add_argument('--load-model', type=str, default=None, help='Path to load a pre-trained model')
    parser.add_argument('--check-env', action='store_true', help='Run environment check before training')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    
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
    
    # Create enhanced logging callback
    logging_callback = EnhancedLoggingCallback(
        save_freq=args.save_freq,
        save_path=args.model_dir,
        name_prefix="bipedal_robot_model"
    )
    
    # Create or load the model
    if args.load_model:
        print(f"Loading pre-trained model from {args.load_model}")
        model = PPO.load(args.load_model, env=env, device="cpu")
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
            max_grad_norm=0.5,
            device="cpu",
        )
    
    # Train the model
    print(f"Starting training for {args.timesteps} timesteps...")
    print(f"Training logs will be saved to {args.log_dir}")
    print(f"Model checkpoints will be saved to {args.model_dir}")
    print("\nTo monitor training with TensorBoard, run:")
    print(f"tensorboard --logdir={args.log_dir}")
    
    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=logging_callback
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