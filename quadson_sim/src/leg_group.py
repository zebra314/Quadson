import pybullet as p
import numpy as np
from src.leg_kinematics import LegKinematics

class LegGroup:
  def __init__(self, robot_id, joint_indices):
    self.robot_id = robot_id
    self.joint_indices = joint_indices
    self.leg_kinematics = LegKinematics()

    # Set initial motor angles
    self.leg_kinematics.motor_angles = [0, np.pi, np.pi/2]
  
  def set_ee_point(self, ee_point):
    self.leg_kinematics.ee_point = ee_point
    motor_angles = self.leg_kinematics.motor_angles

    for joint_index, motor_angle in zip(self.joint_indices, motor_angles):
      p.setJointMotorControl2(
        bodyUniqueId=self.robot_id,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=motor_angle,
        force=800
      )
  
  def set_motor_angles(self, motor_angles):
    self.leg_kinematics.motor_angles = motor_angles

    for joint_index, motor_angle in zip(self.joint_indices, motor_angles):
      p.setJointMotorControl2(
        bodyUniqueId=self.robot_id,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=motor_angle,
        force=800
      )
