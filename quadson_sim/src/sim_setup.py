import pybullet as p
import pybullet_data
import time

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
