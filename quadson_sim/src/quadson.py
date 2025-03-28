import pybullet as p
import numpy as np
from src.config import Config
from src.interface import Interface
from src.leg_group import LegGroup
from src.body_kinematics import BodyKinematics
from src.gait import TrajectoryPlanner

class Quadson:
  def __init__(self, interface: Interface):
    self.robot_id = p.loadURDF("../assets/whole_body/urdf/quadson_modified.urdf",
                                      basePosition=[0, 0, 0.35],
                                      useFixedBase=False)
    self.config = Config()
    self.body_kinematics = BodyKinematics()
    self.trajectory_planner = TrajectoryPlanner(gait_type='trot')
    self.interface = interface
    self.joint_dict = self.setup_joint_dict()
    self.leg_group_dict = self.setup_leg_group()
    self.setup_colors()

  def setup_joint_dict(self):
    """
    Return a dictionary of joint names and their indices
    """
    joint_dict = {
      p.getJointInfo(self.robot_id, i)[1].decode("utf-8"): i
      for i in range(p.getNumJoints(self.robot_id))
    }
    return joint_dict
  
  def setup_colors(self):
    base_color = [0.2, 0.2, 0.2, 1]
    shoulder_color = [0.8, 0.5, 0.2, 1]
    leg_color = [0.7, 0.7, 0.7, 1]
    default_color = [0.2, 0.2, 0.2, 1] # Dark gray for anything else

    # Set base link color
    p.changeVisualShape(self.robot_id, -1, rgbaColor=base_color)

    # Iterate through all links
    for joint_name, joint_index in self.joint_dict.items():
      if 'joint0' in joint_name.lower():
        color = shoulder_color
      elif 'joint' in joint_name.lower():
        color = leg_color
      else:
        color = default_color

      p.changeVisualShape(self.robot_id, joint_index, rgbaColor=color)

  def setup_leg_group(self):
    """
    Initialize leg groups dynamically from joint dictionary.
    Return a dictionary of leg name and the leg group object.
    """
    leg_group_dict = {}
    for leg_name, leg_joints in self.config.joint_dict.items():
      if not all(joint in self.joint_dict for joint in leg_joints):
        print(f"[Warning] Skipping {leg_name} leg group. Some joints are missing in URDF.")
        continue
      joint_indices = [self.joint_dict[joint_name] for joint_name in leg_joints]
      leg_group_dict[leg_name] = LegGroup(self.robot_id, joint_indices)
    
    return leg_group_dict

  def update(self):
    self.interface.send_cmd()
    self.input_dict = self.interface.output_dict[self.interface.target]

    handlers = {
      'motor': self._update_motor,
      'orientation': self._update_orientation,
    }
    handlers[self.interface.target]()

  def step(self, time: float):
    ee_points = self.trajectory_planner.get_trajectory(time)
    for leg_name, ee_point in ee_points.items():
      self.leg_group_dict[leg_name].set_ee_point(ee_point)

  def _update_motor(self):
    base_angles = np.array([0, np.pi, np.pi/2])
    for leg_name in self.config.legs:
      motor_angles = base_angles - np.array(self.input_dict[leg_name])
      self.leg_group_dict[leg_name].set_motor_angles(motor_angles)

  def _update_orientation(self):
    [roll, pitch, yaw] = [value for _, value in self.input_dict.items()]
    self.body_kinematics.update_body_pose(roll, pitch, yaw)
    ee_points = self.body_kinematics.get_ee_points()
    for leg_name, ee_point in zip(self.config.legs, ee_points):
      self.leg_group_dict[leg_name].set_ee_point(ee_point)

  def _update_ee_offset(self) -> None:
    self.ee_offset = self.input_dict
    ee_points = self.trajectory_planner.get_trajectory(self.time_step)
    for leg_name, ee_point in ee_points.items():
      offset = self.ee_offset[leg_name]
      self.leg_group_dict[leg_name].set_ee_point(ee_point+offset)

  def get_observation(self) -> Dict:
    linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
    linear_acc = [
        (linear_vel[0] - self.last_linear_vel[0]) / self.time_step,
        (linear_vel[1] - self.last_linear_vel[1]) / self.time_step,
        (linear_vel[2] - self.last_linear_vel[2]) / self.time_step
    ]
    self.last_linear_vel = linear_vel
    pos, ori = p.getBasePositionAndOrientation(self.robot_id)
    euler_ori = p.getEulerFromQuaternion(ori)

    digits = 5
    obs = {}
    obs['position'] = self.round_tuple(pos, digits)
    obs['linear_vel'] = self.round_tuple(linear_vel, digits)
    obs['linear_acc'] = self.round_tuple(linear_acc, digits)
    obs['angular_vel'] = self.round_tuple(angular_vel, digits)
    obs['euler_ori'] = self.round_tuple(euler_ori, digits)

    return obs
  
  def round_tuple(self, t, digits):
    return tuple(round(x, digits) for x in t)
