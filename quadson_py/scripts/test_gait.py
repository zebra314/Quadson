import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import pybullet as p
import pybullet_data
import time
from src.sim.quadson import Quadson

def main():
  dt = 1 / 240
  current_time = 0.0
  
  p.connect(p.GUI) # (GUI for visualization, DIRECT for headless)
  p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
  p.resetSimulation()
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.setGravity(0, 0, -9.81)
  p.setTimeStep(dt)
  p.loadURDF("plane.urdf")
  
  quadson = Quadson()
  while True:
    quadson.step(current_time)
    p.stepSimulation()
    current_time += dt
    time.sleep(dt)

if __name__ == "__main__":
  main()
