import numpy as np
from src.config import Config
from typing import Dict, List

class GaitGenerator:
  def __init__(self, gait_type='trot'):
    valid_gait_types = ['trot', 'walk', 'run']
    if gait_type not in valid_gait_types:
      print(f"[Warning] Invalid gait type: {gait_type}, using trot as default")
      gait_type = 'trot'

    self.config = Config()
    self.gait_type = gait_type
    self.leg_phase_offset = self.generate_phase_offset()
  
  def generate_phase_offset(self) -> Dict[str, float]:
    if self.gait_type == 'trot':
      return {
        'fl': 0.0,
        'fr': 0.5,
        'rl': 0.0,
        'rr': 0.5,
      }
    elif self.gait_type == 'walk':
      return {
        'fl': 0.0,
        'fr': 0.25,
        'rl': 0.5,
        'rr': 0.75,
      }

  def get_phase(self, time: float) -> Dict[str, float]:
    phase = {}
    cycle_progress = (time % self.config.gait_cycle_time) / self.config.gait_cycle_time
    for leg_name in self.config.legs:
      phase[leg_name] = (cycle_progress + self.leg_phase_offset[leg_name]) % 1.0
    return phase

class TrajectoryPlanner:
  def __init__(self, gait_type='trot'):
    self.config = Config()
    self.gait_generator = GaitGenerator(gait_type)

  def get_trajectory(self, time: float) -> Dict[str, List[float]]:
    phase = self.gait_generator.get_phase(time)
    trajectory = {}
    for leg_name in self.config.legs:
      direction = 1 if leg_name in ['fl', 'fr'] else -1
      # Stance phase
      if phase[leg_name] < 0.5:
        t = phase[leg_name] / 0.5  # 將 0-0.5 映射到 0-1
        p0 = (direction * self.config.step_length, -18.0)  # 起點
        p1 = (direction * self.config.step_length / 2, -18.0)  # 控制點
        p2 = (0.0, -18.0)  # 終點
        x, y = self.quadratic_bezier(t, p0, p1, p2)
      # Swing phase
      else:
        t = (phase[leg_name] - 0.5) / 0.5  # 將 0.5-1 映射到 0-1
        p0 = (0.0, -18.0)  # 起點
        p1 = (0.0, -18.0 + self.config.step_length / 2)  # 控制點 1
        p2 = (direction * self.config.step_length / 2, -18.0 + self.config.step_length / 2)  # 控制點 2
        p3 = (direction * self.config.step_length, -18.0)  # 終點
        x, y = self.cubic_bezier(t, p0, p1, p2, p3)
      z = 0.0
      trajectory[leg_name] = [x, y, z]
    return trajectory
  
  def quadratic_bezier(self, t, p0, p1, p2):
    """二次貝茲曲線計算"""
    x = (1 - t)**2 * p0[0] + 2 * (1 - t) * t * p1[0] + t**2 * p2[0]
    y = (1 - t)**2 * p0[1] + 2 * (1 - t) * t * p1[1] + t**2 * p2[1]
    return x, y

  def cubic_bezier(self, t, p0, p1, p2, p3):
    """三次貝茲曲線計算"""
    x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
    y = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] + 3 * (1 - t) * t**2 * p2[1] + t**3 * p3[1]
    return x, y