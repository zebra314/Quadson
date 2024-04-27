from time import sleep
import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)

# Setup environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Plane
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

p.setAdditionalSearchPath("./python_scripts/urdf/")
quadbotId = p.loadURDF("quadbot.urdf")
# p.setAdditionalSearchPath("./python_scripts/meshes/")
# p.setRealTimeSimulation(0)
# startPos = [0,0,1]
# startOrn = p.getQuaternionFromEuler([0,0,0])

p.setGravity(0,0,-9.81)

p.setRealTimeSimulation(1)

useRealTimeSimulation = True
while p.isConnected():
  p.setGravity(0, 0, -10)
  if (useRealTimeSimulation):
    sleep(0.01)  # Time in seconds.
  else:
    p.stepSimulation()
