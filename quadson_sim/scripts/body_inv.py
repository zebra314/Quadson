import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import pybullet as p
import pybullet_data
import time
from src.quadson import *

def setup_bullet():
  p.connect(p.GUI) # (GUI for visualization, DIRECT for headless)
  p.resetSimulation()
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.setGravity(0, 0, -9.81)
  p.setTimeStep(1/240)
  p.loadURDF("plane.urdf")

def main():
  setup_bullet()
  quadson = Quadson()
  while True:
    quadson.slider_update()
    p.stepSimulation()
    time.sleep(1 / 240)

if __name__ == "__main__":
  main()
