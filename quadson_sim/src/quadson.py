import pybullet as p

class Quadson:
  def __init__(self):
    self.robot_id = p.loadURDF("../assets/whole_body/urdf/quadson_modified.urdf", 
                                      basePosition=[0, 0, 0.35],
                                      useFixedBase=False)
    
    self.num_joints = p.getNumJoints(self.robot_id)
    self.joint_dict = self.get_joint_dict()
    self.setup_closed_chain()
    self.setup_colors()
    # self.setup_motors()

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

    fl_link4 = self.joint_dict['fl_joint4']
    fr_link4 = self.joint_dict['fr_joint4']
    rl_link4 = self.joint_dict['rl_joint4']
    rr_link4 = self.joint_dict['rr_joint4']

    fl_link4_dummy = self.joint_dict['fl_joint5']
    fr_link4_dummy = self.joint_dict['fr_joint5']
    rl_link4_dummy = self.joint_dict['rl_joint5']
    rr_link4_dummy = self.joint_dict['rr_joint5']

    pair_links = [[fl_link4, fl_link4_dummy],
                  [fr_link4, fr_link4_dummy],
                  [rl_link4, rl_link4_dummy],
                  [rr_link4, rr_link4_dummy]]
    
    for links in pair_links:
      link4 = links[0]
      link4_dummy = links[1]

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


  def setup_motors(self):
    for joint_index in range(self.num_joints):
      joint_info = p.getJointInfo(self.robot_id, joint_index)
      print(joint_info)
      parent_index = joint_info[16]