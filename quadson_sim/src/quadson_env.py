import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from src.quadson import Quadson
from src.interface import Interface
import pybullet as p
import gymnasium as gym

class QuadsonEnv(gym.Env):
  def __init__(self):
    super().__init__()

    p.connect(p.GUI)  # use p.DIRECT when training
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)

    self.interface = Interface(type="model", target="ee_offset")
    self.robot = Quadson(self.interface)

    # Observation space
    # body_state:
    #     3D position (x, y, z) +
    #     Orientation (roll, pitch, yaw) +
    #     Linear velocity (x, y, z) +
    #     Angular velocity (x, y, z) = 13
    # joint_state:
    #     12 joints × (position + velocity) = 24
    # leg phases:
    #   sin(phase_LF) +
    #   cos(phase_LF) +
    #   sin(phase_RF) +
    #   cos(phase_RF) +
    #   sin(phase_LH) +
    #   cos(phase_LH) +
    #   sin(phase_RH) +
    #   cos(phase_RH) = 8
    # Total = 45

    # 因為 phase 是週期性資訊，直接用 phase 在 0~1 之間會讓策略很難學習「相位相近」的含義。
    # 用 [sin(2πϕ), cos(2πϕ)] 可以幫助策略理解 phase 週期性！
    self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(45,), dtype=np.float32)
    
    # x, y, z offset in four legs
    self.action_space = spaces.Box(low=-3, high=3, shape=(12,), dtype=np.float32)

  def reset(self):
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    self.robot = Quadson(self.interface)
    obs = self._get_obs()
    return obs

  def step(self, action):
    cmd_dict = ...(action)
    self.interface.send_cmd(cmd_dict)
    self.robot.update()
    p.stepSimulation()

    obs = self._get_obs()
    reward = self._get_reward()
    done = self._check_done()
    info = {}

    return obs, reward, done, info

  def _get_obs(self):
    obs = self.robot.get_observation()
    return np.concatenate([obs['position'], obs['euler_ori'], obs['linear_vel'], obs['angular_vel']])
  

  def _get_reward(self):
    # 根據任務定義 reward（走得遠？平衡？穩定？）
    return ...

  def _check_done(self):
    # 例如：跌倒了就結束
    return ...
