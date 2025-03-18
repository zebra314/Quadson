import pybullet as p
import numpy as np
from src.leg_group import LegGroup
from src.body_kinematics import BodyKinematics

class Quadson:
  def __init__(self):
    self.robot_id = p.loadURDF("../assets/whole_body/urdf/quadson_modified.urdf", 
                                      basePosition=[0, 0, 0.35],
                                      useFixedBase=False)
    self.generate_joint_dict()
    self.setup_closed_chain()
    self.setup_colors()
    self.setup_leg_groups()
    self.setup_gui_interface()
    self.body_kinematics = BodyKinematics()

  def generate_joint_dict(self):
    """
    Return a dictionary of joint names and their indices
    """
    self.joint_dict = {
      p.getJointInfo(self.robot_id, i)[1].decode("utf-8"): i
      for i in range(p.getNumJoints(self.robot_id))
    }

  def setup_closed_chain(self):
    """
    The child link of joint 4 should connect back to the child link of joint 0 in each leg
    """
    robot_id = self.robot_id

    pairs = [
      ("fl_joint4", "fl_joint5"),
      ("fr_joint4", "fr_joint5"),
      ("rl_joint4", "rl_joint5"),
      ("rr_joint4", "rr_joint5")
    ]
    
    for joint4_name, joint5_name in pairs:
      # Check the link exists
      if not joint4_name in self.joint_dict or not joint5_name in self.joint_dict:
        raise Exception("Couldn't find the links to attach, please check the model")
      
      # Get the link inedx
      link4 = self.joint_dict[joint4_name]
      link4_dummy = self.joint_dict[joint5_name]

      # Create the revolute constraint
      p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=link4,
        childBodyUniqueId=robot_id,
        childLinkIndex=link4_dummy,
        jointType=p.JOINT_FIXED,
        jointAxis=[1, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
      )

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

  def setup_leg_groups(self):
    """
    Initialize leg groups dynamically from joint dictionary.
    Return a dictionary of leg name and the leg group object.
    """
    self.leg_groups = {}

    for leg in ["fl", "fr", "rl", "rr"]:
      joint_names = [f"{leg}_joint0", f"{leg}_joint1", f"{leg}_joint4"]
      if all(j in self.joint_dict for j in joint_names):
        joint_indices = [self.joint_dict[j] for j in joint_names]
        self.leg_groups[leg] = LegGroup(self.robot_id, joint_indices)
      else:
        raise Exception(f"Skipping {leg} leg group. Some joints are missing in URDF.")

  def setup_gui_interface(self):
    legs = {
      "fl": ["fl_joint0", "fl_joint1", "fl_joint4"],  # Front-left
      "fr": ["fr_joint0", "fr_joint1", "fr_joint4"],  # Front-right
      "rl": ["rl_joint0", "rl_joint1", "rl_joint4"],  # Rear-left
      "rr": ["rr_joint0", "rr_joint1", "rr_joint4"]   # Rear-right
    }

    sliders = {}
    for leg, leg_joints in legs.items():
      sliders[leg] = {}
      for joint_name in leg_joints:
        slider_id = p.addUserDebugParameter(joint_name, -np.pi, np.pi, 0)
        sliders[leg][joint_name] = slider_id
    self.sliders = sliders

  def update_pose(self):
    for leg, sliders in self.sliders.items():
      angles = []
      for joint_name, slider_id in sliders.items():
        angle = p.readUserDebugParameter(slider_id)
        angles.append(angle)
      self.leg_groups[leg].set_motor_angles(angles)
