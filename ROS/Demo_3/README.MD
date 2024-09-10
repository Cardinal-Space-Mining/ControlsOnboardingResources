# Fuzzyball - Autonomated

**THIS IS A WORK IN PROGRESS DEMO (NOT COMPLETED)**

In this demo, you will create a ROS program to control the test chassis/robot autonomously. This demo is a much more basic version of how our competition robot will function. In Lunabotics, you can score more points for your robot running autonomously using many sensors including lidar and cameras. We will not include these into the demo as they can be complicated and can be fragile/expensive.

The premise of the demo is for the chassis/robot to navigate a predefined course/objectives autonomously. As we do not have a predesigned course, there will **not** be a solution, but instead you will be provided a template to complete this challenge. This will provide a real sense of how we work designing and programming our package for competition.

## Challenge
Unlike Demo 2, this demo will be a challenge for you to complete. All code from Demo2 will be included accept for the joystick controls. They, instead will be reduced for start/stop commands to the robot for control of autonomy. This is how competitions will run, we start/stop autonomy as we need.

All autonomy should be controlled from the robot itself, no computations should be completed on the your laptop. As stated, the controller will be used to start/stop autonomy.

The `start` button will switch between `teleop` and `autonomy` modes. You will need to work on both `control/robot` nodes for this demo.

Note: without Lidar/camera data you will not use any localization, instead you should use timer objects. You will also probably need to use math to determine times and speeds of motors.

**DO NOT RUN MOTORS AT FULL SPEED DURING THIS**, this is dangerous and not safe practice. Instead, keep motor speeds `50-75% max`.

### References
https://github.com/vanderbiltrobotics/ros_phoenix/tree/foxy

https://github.com/CrossTheRoadElec/Phoenix6-Linux-Example