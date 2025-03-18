import pybullet as p

class Actuator:
  def __init__(self, robot_id, joint_index):
    self.robot_id = robot_id
    self.joint_index = joint_index

  def set_motor_angle(self, motor_angle):
    p.setJointMotorControl2(
      bodyUniqueId=self.robot_id,
      jointIndex=self.joint_index,
      controlMode=p.POSITION_CONTROL,
      targetPosition=motor_angle,
      force=800
    )
