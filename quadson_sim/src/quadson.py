import pybullet as p
import numpy as np
from src.leg_group import LegGroup

class Quadson:
  def __init__(self):
    self.robot_id = p.loadURDF("../assets/whole_body/urdf/quadson_modified.urdf", 
                                      basePosition=[0, 0, 0.35],
                                      useFixedBase=False)
    self.config = Config()
    self.joint_dict = self.setup_joint_dict()
    self.leg_group_dict = self.setup_leg_group()
    self.slider_dict = self.setup_slider_interface()
    self.setup_colors()
    self.motor_angle_dict = {}

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

  def setup_slider_interface(self):
    sliders = {}
    for leg_name, leg_motors in self.config.motor_dict.items():
      for motor_name in leg_motors:
        slider_id = p.addUserDebugParameter(motor_name, -np.pi, np.pi, 0)
        sliders[motor_name] = slider_id

    return sliders

  def slider_update(self):
    # Get new angle from sliders
    for leg_name, leg_motors in self.config.motor_dict.items():
      motor_angles = []
      for motor_name in leg_motors:
        slider_inedx = self.slider_dict[motor_name]
        angle = p.readUserDebugParameter(slider_inedx)
        motor_angles.append(angle)
      motor_angles = np.array([0, np.pi, np.pi/2]) - np.array(motor_angles)
      self.motor_angle_dict[leg_name] = motor_angles

    self.update()

  def update(self):
    # Update the leg groups
    for leg_name, angles in self.motor_angle_dict.items():
      self.leg_group_dict[leg_name].set_motor_angles(angles)

class Config:
  """
  A class to store the joint names for the legs.
  """
  def __init__(self):
    # Private
    ## Front left leg
    self._fl = "fl"
    self._fl_joint0 = "fl_joint0"
    self._fl_joint1 = "fl_joint1"
    self._fl_joint2 = "fl_joint2"
    self._fl_joint4 = "fl_joint4"
    self._fl_joint5 = "fl_joint5"
    self._fl_joints = [self._fl_joint0, self._fl_joint1, self._fl_joint2, self._fl_joint4, self._fl_joint5]
    self._fl_motors = [self._fl_joint0, self._fl_joint1, self._fl_joint5]
    
    ## Front right leg
    self._fr = "fr"
    self._fr_joint0 = "fr_joint0"
    self._fr_joint1 = "fr_joint1"
    self._fr_joint2 = "fr_joint2"
    self._fr_joint4 = "fr_joint4"
    self._fr_joint5 = "fr_joint5"
    self._fr_joints = [self._fr_joint0, self._fr_joint1, self._fr_joint2, self._fr_joint4, self._fr_joint5]
    self._fr_motors = [self._fr_joint0, self._fr_joint1, self._fr_joint5]
    
    ## Rear left leg
    self._rl = "rl"
    self._rl_joint0 = "rl_joint0"
    self._rl_joint1 = "rl_joint1"
    self._rl_joint2 = "rl_joint2"
    self._rl_joint4 = "rl_joint4"
    self._rl_joint5 = "rl_joint5"
    self._rl_joints = [self._rl_joint0, self._rl_joint1, self._rl_joint2, self._rl_joint4, self._rl_joint5]
    self._rl_motors = [self._rl_joint0, self._rl_joint1, self._rl_joint5]
    
    ## Rear right leg
    self._rr = "rr"
    self._rr_joint0 = "rr_joint0"
    self._rr_joint1 = "rr_joint1"
    self._rr_joint2 = "rr_joint2"
    self._rr_joint4 = "rr_joint4"
    self._rr_joint5 = "rr_joint5"
    self._rr_joints = [self._rr_joint0, self._rr_joint1, self._rr_joint2, self._rr_joint4, self._rr_joint5]
    self._rr_motors = [self._rr_joint0, self._rr_joint1, self._rr_joint5]

    # Public
    self.legs = [self._fl, self._fr, self._rl, self._rr]
    self.joint_dict = {self._fl:self._fl_joints,
                       self._fr:self._fr_joints,
                       self._rl:self._rl_joints,
                       self._rr:self._rr_joints}
    self.motor_dict = {self._fl:self._fl_motors,
                       self._fr:self._fr_motors,
                       self._rl:self._rl_motors,
                       self._rr:self._rr_motors}
