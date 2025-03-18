import numpy as np
from src.leg_kinematics import LegKinematics
from src.actuator import Actuator

class LegGroup:
  def __init__(self, robot_id, joint_indices):
    self.robot_id = robot_id
    self.actuators = [Actuator(robot_id, joint_index) for joint_index in joint_indices]
    self.leg_kinematics = LegKinematics()
    self.leg_kinematics.motor_angles = [0, np.pi, np.pi/2] # initial motor angles

  def set_ee_point(self, ee_point):
    self.leg_kinematics.ee_point = ee_point
    motor_angles = self.leg_kinematics.motor_angles
    for actuator, motor_angle in zip(self.actuators, motor_angles):
      actuator.set_motor_angle(motor_angle)
  
  def set_motor_angles(self, motor_angles):
    self.leg_kinematics.motor_angles = motor_angles
    for actuator, motor_angle in zip(self.actuators, motor_angles):
      actuator.set_motor_angle(motor_angle)

  def get_ee_point(self):
    return self.leg_kinematics.ee_point
  
  def get_motor_angles(self):
    return self.leg_kinematics.motor_angles
  