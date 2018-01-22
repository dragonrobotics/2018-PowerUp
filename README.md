<img src="logo.png" align="left" height=150 />

# Team 5002 Power Up
[![Build Status](https://travis-ci.org/dragonrobotics/2018-PowerUp.svg?branch=master)](https://travis-ci.org/dragonrobotics/2018-PowerUp)

This is the official repository for Team 5002 Dragon Robotics' 2018 FIRST Power
Up code. Our code is written in Python using  [RobotPy](https://robotpy.github.io/).  The directory structure is laid out as
such:
- The `lift` folder contains the code for both the claw and the RD4B subsystems
for manipulating cubes.
- The `sensors` folder contains code for interfacing with sensors, such as the
I2C ultrasonic sensor.
- The `swerve` folder contains code for the swerve drive, including code to
operate each module and also calculations.
- The `tests` folder contains tests for our code, including simulation tests to
make sure the code works as intended before pushing it to the robot.
- The `vision` folder contains vision code for the robot.

All code should be well commented and documented. The HTML documentation can
be found at https://dragonrobotics.github.io/2018-PowerUp.
