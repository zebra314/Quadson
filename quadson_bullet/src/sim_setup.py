import pybullet
import pybullet_data
import time

def setup_bullet():
  pybullet.connect(pybullet.GUI) # (GUI for visualization, DIRECT for headless)
  pybullet.resetSimulation()
  pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
  pybullet.setGravity(0, 0, -9.81)
  pybullet.setTimeStep(1/240)

def run_simulation():
  while True:
    pybullet.stepSimulation()
    time.sleep(1 / 240)

def setup_env():
  # Create a plane
  plane_id = pybullet.loadURDF("plane.urdf")
  return plane_id

class Quadson:
  def __init__(self):
    self.robot_id = pybullet.loadURDF("../assets/urdf/quadson_modified.urdf", 
                                      basePosition=[0, 0, 0.35],
                                      useFixedBase=False)
    
    # Fix the robot to the world
    constraint_id = pybullet.createConstraint(
      parentBodyUniqueId=self.robot_id, 
      parentLinkIndex=-1,
      childBodyUniqueId=-1, # 固定到世界座標
      childLinkIndex=-1,
      jointType=pybullet.JOINT_FIXED,
      jointAxis=[0, 0, 0], 
      parentFramePosition=[0, 0, 0],  
      childFramePosition=[0, 0, 0.35]
    )
    
    self.num_joints = pybullet.getNumJoints(self.robot_id)

    self.joint_dict = self.get_joint_dict()
    # self.disable_inner_collision()
    self.setup_closed_chain()
    self.setup_motors()

  def get_joint_dict(self):
    joint_dict = {}
    for joint_index in range(self.num_joints):
      joint_info = pybullet.getJointInfo(self.robot_id, joint_index)
      joint_name = joint_info[1].decode("utf-8")
      joint_dict[joint_name] = joint_index
    return joint_dict

  def disable_inner_collision(self):
    for joint_index in range(self.num_joints):
      joint_info = pybullet.getJointInfo(self.robot_id, joint_index)
      child_index = joint_index
      parent_index = joint_info[16]
      pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, child_index, parent_index, 0)
  
  def setup_closed_chain(self):
    # The child link of joint 4 should connect back to the child link of joint 0 in each leg
    parent_index = self.joint_dict["fl_joint0"]
    child_index = self.joint_dict["fl_joint4"]
    print("\n")
    print("child_index", child_index)
    print("parent_index", parent_index)
    print("\n")
    pybullet.createConstraint(
      parentBodyUniqueId=self.robot_id,
      parentLinkIndex=parent_index,
      childBodyUniqueId=self.robot_id,
      childLinkIndex=child_index,
      jointType=pybullet.JOINT_POINT2POINT,
      jointAxis=[0, 0, 1],
      # parentFramePosition=[0, 0.01862, 0.13630],
      # parentFramePosition=[0, 0.13630, 0.01862],
      # parentFramePosition=[0.01862, 0, 0.13630],
      # parentFramePosition=[0.13630, 0, 0.01862],
      # parentFramePosition=[0.01862, 0.13630, 0],
      parentFramePosition=[0.13630, 0.01862, 0],
      childFramePosition=[0, 0, 0]
    )

  def get_motor_dict(self):

    
    leg_configs = {
      # "fr": {"motor0": joint_dict["fr_joint0"], "motor1": joint_dict["fr_joint1"], "motor5": joint_dict["fr_joint5"]},    # Front-right
      # "fl": {"motor0": joint_dict["fl_joint0"], "motor1": joint_dict["fl_joint1"], "motor5": joint_dict["fl_joint5"]},    # Front-left
      # "rr": {"motor0": joint_dict["br_joint0"], "motor1": joint_dict["br_joint1"], "motor5": joint_dict["br_joint5"]},    # Rear-right
      # "rl": {"motor0": joint_dict["bl_joint0"], "motor1": joint_dict["bl_joint1"], "motor5": joint_dict["bl_joint5"]}   # Rear-left
    }
    return leg_configs

  def setup_motors(self):
    for joint_index in range(self.num_joints):
      joint_info = pybullet.getJointInfo(self.robot_id, joint_index)
      print(joint_info)
      parent_index = joint_info[16]