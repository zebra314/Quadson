import pybullet as p
import time
import math

# Connect to PyBullet
p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode
p.setGravity(0, 0, -9.81)

# Load the URDF
robot_id = p.loadURDF("../assets/single_leg/urdf/single_leg_modified.urdf", useFixedBase=True)

# Identify joint indices (based on URDF order)
joint1_index = 0  # joint1 (link0_link to link1_link)
joint4_index = 3  # joint4 (link3_link to link4_link)

# Verify joint names (optional)
for i in range(p.getNumJoints(robot_id)):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {joint_info[1].decode('utf-8')}")

# Define control parameters
max_angle = 30 * (math.pi / 180)  # 30 degrees in radians (0.5236)
frequency = 1.0  # Oscillation frequency in Hz
amplitude = max_angle  # Max angle to stay within ±30°

# Add fixed constraint to close the chain (from previous solution)
link4_index = 3       # link4_link
link4_dummy_index = 4 # link4_dummy_link

p.createConstraint(
    parentBodyUniqueId=robot_id,
    parentLinkIndex=link4_index,
    childBodyUniqueId=robot_id,
    childLinkIndex=link4_dummy_index,
    jointType=p.JOINT_FIXED,
    jointAxis=[1, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0]
)

# # Enable joint motors
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=joint1_index,
    controlMode=p.POSITION_CONTROL,
    targetPosition=0,
    force=500  # Adjust force as needed
)
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=joint4_index,
    controlMode=p.POSITION_CONTROL,
    targetPosition=0,
    force=500
)

# Simulation loop
time_step = 1.0 / 240.0  # PyBullet default time step
p.setRealTimeSimulation(0)  # Step simulation manually

for t in range(100000):
    current_time = t * time_step
    
    # Calculate target positions within ±30° using a sine wave
    target_angle1 = amplitude * math.sin(2 * math.pi * frequency * current_time)
    target_angle4 = amplitude * math.sin(2 * math.pi * frequency * current_time + math.pi)  # Phase offset for variety

    # Ensure angles stay within ±30° (redundant due to amplitude, but good practice)
    target_angle1 = max(-max_angle, min(max_angle, target_angle1))
    target_angle4 = max(-max_angle, min(max_angle, target_angle4))

    # Apply position control
    # p.setJointMotorControl2(
    #     bodyUniqueId=robot_id,
    #     jointIndex=joint1_index,
    #     controlMode=p.POSITION_CONTROL,
    #     targetPosition=target_angle1,
    #     force=500
    # )
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=joint4_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_angle4,
        force=800
    )

    # Step the simulation
    p.stepSimulation()
    time.sleep(time_step)  # Slow down for visualization

# Disconnect when done
p.disconnect()