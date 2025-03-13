import pybullet as p
from src.leg_group import LegGroup
from src.body_kinematics import BodyKinematics

class Quadson:
  def __init__(self):
    self.robot_id = p.loadURDF("../assets/whole_body/urdf/quadson_modified.urdf", 
                                      basePosition=[0, 0, 0.35],
                                      useFixedBase=True)
    self.num_joints = p.getNumJoints(self.robot_id)
    self.joint_dict = self.get_joint_dict()
    self.setup_closed_chain()
    self.setup_colors()
    self.setup_leg_groups()
    self.body_kinematics = BodyKinematics()

  def get_joint_dict(self):
    joint_dict = {}
    for joint_index in range(self.num_joints):
      joint_info = p.getJointInfo(self.robot_id, joint_index)
      joint_name = joint_info[1].decode("utf-8")
      joint_dict[joint_name] = joint_index
      # print(joint_name, joint_index)
    return joint_dict

  def setup_closed_chain(self):
    # The child link of joint 4 should connect back to the child link of joint 0 in each leg
    robot_id = self.robot_id

    pairs = [
      ("fl_joint4", "fl_joint5"),
      ("fr_joint4", "fr_joint5"),
      ("rl_joint4", "rl_joint5"),
      ("rr_joint4", "rr_joint5")
    ]
    
    for joint4_name, joint5_name in pairs:
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
    legs = {
      "fl": ["fl_joint0", "fl_joint1", "fl_joint5"],  # Front-left
      "fr": ["fr_joint0", "fr_joint1", "fr_joint5"],  # Front-right
      "rl": ["rl_joint0", "rl_joint1", "rl_joint5"],  # Rear-left
      "rr": ["rr_joint0", "rr_joint1", "rr_joint5"]   # Rear-right
    }

    self.leg_groups = {}
    for leg_name, leg_joints in legs.items():
      joint_indices = [self.joint_dict[joint_name] for joint_name in leg_joints]
      leg_group = LegGroup(self.robot_id, joint_indices)
      self.leg_groups[leg_name] = leg_group
