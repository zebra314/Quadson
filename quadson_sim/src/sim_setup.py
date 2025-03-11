import pybullet as p
import pybullet_data
import time
import numpy as np

def setup_bullet():
  p.connect(p.GUI) # (GUI for visualization, DIRECT for headless)
  p.resetSimulation()
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.setGravity(0, 0, -9.81)
  p.setTimeStep(1/240)

def run_simulation():
  while True:
    p.stepSimulation()
    time.sleep(1 / 240)

def setup_env():
  # Create a plane
  plane_id = p.loadURDF("plane.urdf")
  return plane_id

class Quadson:
  def __init__(self):
    self.robot_id = p.loadURDF("../assets/urdf/quadson_modified.urdf", 
                                      basePosition=[0, 0, 0.35],
                                      useFixedBase=False)
    
    self.num_joints = p.getNumJoints(self.robot_id)
    self.joint_dict = self.get_joint_dict()
    self.disable_inner_collision()
    self.setup_closed_chain()
    self.setup_motors()
    # self.fixed_robot()
  
  def fixed_robot(self):
    # Fix the robot to the world
    constraint_id = p.createConstraint(
      parentBodyUniqueId=self.robot_id, 
      parentLinkIndex=-1,
      childBodyUniqueId=-1, # 固定到世界座標
      childLinkIndex=-1,
      jointType=p.JOINT_FIXED,
      jointAxis=[0, 0, 0], 
      parentFramePosition=[0, 0, 0],  
      childFramePosition=[0, 0, 0.35]
    )

  def get_joint_dict(self):
    joint_dict = {}
    for joint_index in range(self.num_joints):
      joint_info = p.getJointInfo(self.robot_id, joint_index)
      joint_name = joint_info[1].decode("utf-8")
      joint_dict[joint_name] = joint_index
    return joint_dict

  def disable_inner_collision(self):
    for joint_index in range(self.num_joints):
      joint_info = p.getJointInfo(self.robot_id, joint_index)
      child_index = joint_index
      parent_index = joint_info[16]
      p.setCollisionFilterPair(self.robot_id, self.robot_id, child_index, parent_index, 0)
  
  def setup_closed_chain(self):
    # The child link of joint 4 should connect back to the child link of joint 0 in each leg
    robot_id = self.robot_id
    link0_index = 10
    link4_index = 14

    # Get initial world position of bl_link4's origin
    position4, _ = p.getLinkState(robot_id, link4_index)[0:2]
    position4 = np.array(position4)

    # Get world transform of bl_link0
    position0, orientation0 = p.getLinkState(robot_id, link0_index)[0:2]
    position0 = np.array(position0)

    # Compute the inverse transform of bl_link0
    _, inv_orientation0 = p.invertTransform(position0, orientation0)

    # Compute the relative position (world position of link4's origin relative to link0's origin)
    delta_position = position4 - position0

    # Transform this position into bl_link0's local frame
    pivotInB = p.rotateVector(inv_orientation0, delta_position.tolist())

    # Define pivot point in bl_link4's frame (at origin)
    pivotInA = [0, 0, 0]

    # Define joint axis (z-axis in bl_link0's frame)
    jointAxis = [0, 0, 1]

    # Create the revolute constraint
    constraint_id = p.createConstraint(
      parentBodyUniqueId=robot_id,
      parentLinkIndex=link4_index,
      childBodyUniqueId=robot_id,
      childLinkIndex=link0_index,
      jointType=p.JOINT_FIXED,
      jointAxis=jointAxis,
      parentFramePosition=pivotInA,
      childFramePosition=pivotInB
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
      joint_info = p.getJointInfo(self.robot_id, joint_index)
      # print(joint_info)
      parent_index = joint_info[16]