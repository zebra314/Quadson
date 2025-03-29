import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from matplotlib import pyplot as plt
import pybullet as p
import pybullet_data
import gymnasium as gym
from stable_baselines3.common.callbacks import BaseCallback
from src.quadson import Quadson
from src.interface import Interface
from src.config import Config

class QuadsonEnv(gym.Env):
  def __init__(self):
    super().__init__()

    p.connect(p.GUI)  # use p.DIRECT when training
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setTimeStep(1/240)

    self.interface = Interface(type="model", target="ee_offset")
    self.robot = Quadson(self.interface)
    self.config = Config()

    self.rewards = []

    # Observation space
    # body_state:
    #     3D position (x, y, z) +
    #     Orientation (roll, pitch, yaw) +
    #     Linear velocity (x, y, z) +
    #     Angular velocity (x, y, z) = 12
    # joint_state:
    #     12 joints × (position) = 12 # Add velocity
    # leg phases:
    #   sin(phase_LF) +
    #   cos(phase_LF) +
    #   sin(phase_RF) +
    #   cos(phase_RF) +
    #   sin(phase_LH) +
    #   cos(phase_LH) +
    #   sin(phase_RH) +
    #   cos(phase_RH) = 8
    # Total = 32

    # 因為 phase 是週期性資訊，直接用 phase 在 0~1 之間會讓策略很難學習「相位相近」的含義。
    # 用 [sin(2πϕ), cos(2πϕ)] 可以幫助策略理解 phase 週期性！
    self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(32,), dtype=np.float32)
    
    # x, y, z offset in four legs
    self.action_space = gym.spaces.Box(low=-5, high=5, shape=(12,), dtype=np.float32)
    self.last_action = np.zeros(12)

  def reset(self, *, seed=None, options=None):
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    self.robot = Quadson(self.interface)
    obs = self._get_obs()
    return obs, {}

  def step(self, action):
    cmd_dict = {}
    for i, leg_name in enumerate(self.config.legs):
      start = i * 3
      end = start + 3
      cmd_dict[leg_name] = action[start:end]

    self.robot.update(cmd_dict)
    p.stepSimulation()

    # Fix the camera
    basePos, baseOrn = p.getBasePositionAndOrientation(self.robot.robot_id) # Get model position
    p.resetDebugVisualizerCamera( cameraDistance=0.6, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=basePos) # fix camera onto model

    obs = self._get_obs()
    reward = self._get_reward()
    done = self._check_done()
    info = {}
    terminated = True
    truncated = False
    return obs, reward, done, truncated, info

  def _get_obs(self):
    obs = self.robot.get_observation()
    return obs
  
  def _get_reward(self):
    # 1. Forward velocity (assume x-axis forward)
    lin_vel = self.robot.get_linear_velocity()  # shape (3,)
    forward_vel = lin_vel[0]

    # 2. Penalize roll and pitch (keep body horizontal)
    roll, pitch, _ = self.robot.get_orientation_rpy()
    orientation_penalty = roll**2 + pitch**2  # small if upright

    # 3. Height deviation penalty
    z = self.robot.get_position()[2]
    target_height = 0.2  # around 0.25~0.3m maybe?
    height_penalty = (z - target_height) ** 2

    # 4. Action smoothness (penalize big actions)
    energy_penalty = np.sum(np.square(self.last_action))

    reward = (
      + 1.0 * forward_vel         # encourage forward motion
      - 0.5 * orientation_penalty # penalize tilt
      - 0.3 * height_penalty      # penalize abnormal height
      - 0.01 * energy_penalty     # small penalty to avoid jerky motion
    )

    return reward

  def _check_done(self):
    roll, pitch, _ = self.robot.get_orientation_rpy()
    z = self.robot.get_position()[2]

    if abs(roll) > np.pi / 4 or abs(pitch) > np.pi / 4:  # 45 degrees tilt
      return True

    if z < 0.1:  # robot collapsed
      return True

    # if self.step_counter >= self.max_steps:
    #     return True

    return False

class PlottingCallback(BaseCallback):
    """
    Callback for plotting episode rewards during training.
    """
    def __init__(self, verbose=0):
        super(PlottingCallback, self).__init__(verbose)
        self.episode_rewards = []
        self.moving_avg_rewards = []
        self.window_size = 10  # For moving average
        
    def _on_training_start(self):
        """Initialize the plot when training starts"""
        plt.figure(figsize=(10, 5))
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('Episode')
        self.ax.set_ylabel('Reward')
        self.ax.set_title('Training Reward Over Time')
        plt.show(block=False)
        
        # Initialize reward tracking
        self.current_episode_reward = 0
        
    def _on_step(self):
        """Update reward tracking on each step"""
        # Get reward - handle both single and vectorized environments
        if isinstance(self.locals['rewards'], list) or isinstance(self.locals['rewards'], np.ndarray):
            reward = self.locals['rewards'][0]  # Take first env if vectorized
        else:
            reward = self.locals['rewards']
            
        # Accumulate reward
        self.current_episode_reward += reward
        
        # Check if episode ended
        done = self.locals['dones'] if isinstance(self.locals['dones'], bool) else self.locals['dones'][0]
        
        if done:
            # Store episode reward
            self.episode_rewards.append(self.current_episode_reward)
            
            # Calculate moving average
            if len(self.episode_rewards) >= self.window_size:
                avg = np.mean(self.episode_rewards[-self.window_size:])
                self.moving_avg_rewards.append(avg)
            else:
                self.moving_avg_rewards.append(self.episode_rewards[-1])
            
            # Update plot
            episodes = np.arange(len(self.episode_rewards))
            
            # Clear previous plot
            self.ax.clear()
            
            # Plot raw rewards and moving average
            self.ax.plot(episodes, self.episode_rewards, 'b-', alpha=0.3, label='Episode Reward')
            self.ax.plot(episodes, self.moving_avg_rewards, 'r-', label='Moving Average')
            
            # Add labels
            self.ax.set_xlabel('Episode')
            self.ax.set_ylabel('Reward')
            self.ax.set_title(f'Training Reward (Latest: {self.episode_rewards[-1]:.2f})')
            self.ax.legend()
            
            # Refresh plot
            plt.draw()
            plt.pause(0.01)
            
            # Reset for next episode
            self.current_episode_reward = 0
            
            if self.verbose > 0:
                print(f"Episode {len(self.episode_rewards)}: Reward = {self.episode_rewards[-1]:.2f}")
        
        return True
        
    def _on_training_end(self):
        """Save the final plot when training ends"""
        plt.savefig('training_rewards.png')
        if self.verbose > 0:
            print(f"Training ended. Final plot saved to 'training_rewards.png'")
        plt.close()