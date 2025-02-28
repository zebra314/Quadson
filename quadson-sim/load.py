import os
import pybullet
import pybullet_data
import time

# Connect to PyBullet (GUI for visualization, DIRECT for headless)
pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0, 0, -9.81)
pybullet.setTimeStep(1/240)

# Load the URDF file
startPos = [0, 0, 0.4]
robot = pybullet.loadURDF("./urdf/quadson.urdf", startPos, useFixedBase=False)

# Create a plane
plane = pybullet.loadURDF("./urdf/plane.urdf")

# Keep the simulation running
while True:
    pybullet.stepSimulation()
    time.sleep(1 / 240)
