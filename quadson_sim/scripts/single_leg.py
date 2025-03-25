import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import pybullet as p
import time
import numpy as np
from src.leg_kinematics import LegKinematics

def joint_ctrl(joint_index, target_angle):
  p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=joint_index,
    controlMode=p.POSITION_CONTROL,
    targetPosition=target_angle,
    force=500
  )

kinematics = LegKinematics()
kinematics.motor_angles = [0, np.pi, np.pi/2]

def get_env_angles(theory_angles):
  j1_env = np.pi - theory_angles[1]
  j5_env = np.pi/2 - theory_angles[5]

  j2_env = 1.2406 - (np.pi + theory_angles[2] - theory_angles[1])
  j4_env =  - 1.6833 + (np.pi + theory_angles[5] - theory_angles[4])

  return [j1_env, j2_env, j4_env, j5_env]

def enfore_closure(env_input):
  kinematics.motor_angles = [0, np.pi-env_input[0], np.pi/2-env_input[1]]
  env_angles = get_env_angles(kinematics.angles)

  joint_ctrl(0, env_angles[0])
  joint_ctrl(1, env_angles[1])
  joint_ctrl(3, env_angles[2])
  joint_ctrl(2, env_angles[3])

# Connect to PyBullet
p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode
p.setGravity(0, 0, -9.81)

# Load the URDF
robot_id = p.loadURDF("../assets/single_leg/urdf/single_leg.urdf", useFixedBase=True)

# Verify joint names (optional)
# for i in range(p.getNumJoints(robot_id)):
#   joint_info = p.getJointInfo(robot_id, i)
#   print(f"{i}: {joint_info[1].decode('utf-8')}")

# Simulation loop
time_step = 1.0 / 240.0  # PyBullet default time step
p.setRealTimeSimulation(0)  # Step simulation manually

slider_joint1 = p.addUserDebugParameter('joint1', -np.pi, np.pi, 0)
slider_joint5 = p.addUserDebugParameter('joint5', -np.pi, np.pi, 0)

for t in range(100000):
    bais1 = p.readUserDebugParameter(slider_joint1)
    bais5 = p.readUserDebugParameter(slider_joint5)
    env_input = [bais1, bais5]

    enfore_closure(env_input)

    p.stepSimulation()
    time.sleep(time_step)

p.disconnect()
