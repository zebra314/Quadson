# Quadbot

## Description

## Usage

### Real time control

- Activate the robot

  ```sh
  make
  ```

- Deactivate the robot

  ```sh
  make stop
  ```

- Clean the project

  ```sh
  make clean
  ```

### Simulation

- Install the simulation

  ```sh
  make install
  ```

- Activate the simulation

  ```sh
  make sim
  ```

## Files

```sh
quadbot_ws/
│
├── build/
│ ├── ...
│ .
│ .
│ .
│ └──
│
├── include/
│ ├── actuator.h
│ ├── AHRS.h
│ ├── can_interface.h
│ ├── can_protocol.h
│ ├── leg_group.h
│ ├── quadson.h
│ └── utility.h
│
├── library/
│ ├── ...
│ .
│ .
│ .
│ └──
│
├── source/
│ ├── actuator.cpp
│ ├── AHRS.cpp
│ ├── can_interface.cpp
│ ├── can_protocol.cpp
│ ├── leg_group.cpp
│ ├── main.cpp
│ └── quadson.cpp
│
├── python_scripts/
│ ├── CAN_MotorControl.py
│ └── quad_calc_test.py
│
├── start_can.sh
│
└── stop_can.sh
```

- `actuator.h/actuator.cpp`: Define the basic function for a single actuator, such as enable, disable, set angle, set omega, get information, etc.

- `AHRS.h/AHRS.cpp`: Define the basic function and algorithm for the Attitude and Heading Reference System (AHRS) module. It is used to get the orientation of the robot.

- `main.cpp`: Main file of the project

- `can_interface.h/can_interface.cpp`: Create a can interface class to communicate with the can bus from the computer to the robot.

  ```sh
  Actuator alpha ─┐
  Actuator beta  ───> CAN BUS <───> Raspberry Pi
  Actuator gamma ─┘
  ```

- `can_protocol.h/can_protocol.cpp`: Define the protocol to communicate with the can bus. It includes the message ID(standard or extended), the message type, the message data, etc.

- `leg_group.h/leg_group.cpp`: Define the leg group class to control the leg group of the robot. It unified three actuators and responsible for the inverse kinematics, forward kinematics, gait planning and pid control.

- `quadson.h/quadson.cpp`: Define the main class of the project. It includes the main loop of the robot, the initialization of the robot, it also unified the four leg groups and responsible for sending the command to the actuators via the can bus.

- `utility.h`: Define some utility functions

- `start_can.sh`: Bash script to start the can interface

- `stop_can.sh`: Bash script to stop the can interface

## References

- [Mahony AHRS algorithm implemented by Madgwick](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

  According to the Mahony AHRS algorithm implemented by Madgwick and adapted by Igor Vereninov, provided by Emlid Ltd (c) 2014:

  ```bibtex
  @MISC{madgwick2014,
  author =   {Madgwick, S. and Vereninov, I.},
  title =    {Mahony AHRS algorithm},
  howpublished = {\url{http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/}},
  year = {2014}
  }
  ```

- [PyBullet](http://pybullet.org)

  ```bibtex
  @MISC{coumans2021,
  author =   {Erwin Coumans and Yunfei Bai},
  title =    {PyBullet, a Python module for physics simulation for games, robotics and machine learning},
  howpublished = {\url{http://pybullet.org}},
  year = {2016--2021}
  }
  ```
