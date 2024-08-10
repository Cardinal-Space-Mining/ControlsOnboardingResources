# Fuzzyball - Teleoperated

**THIS IS A WORK IN PROGRESS DEMO (NOT COMPLETED)**

The goal for this Demo is to create a ROS program using the Phoenix 6 API such that using a joystick controller you can drive the test chassis around. Also in this demo, you will/should learn how to collect and log data throughout your run. This is crucial for our club, during our runs and tests we collect data regarding current, temperatures, etc... that we can analyze to determine if there are any issues and how we could fix them. In the case of a motor going into thermal shutdown we can analyze how this happened and normally current is involved.

In this demo, you will create a ROS program to control the test chassis/robot with a controller. In standard practice for our club, we will use a Logitch controller. This form of control is called Teleoperated or Teleop for short. In the Lunabotics competition we are allowed to control our robot with this system. Although we will lose points for this.

This will be a very basic implementation project. In our development, some practices may vary including how ROS messages may be sent.

There will **not** be a solution, but instead you will be provided a template to complete this challenge. This will provide a real sense of how we work designing and programming our package for competition.

Note: It is encouraged to use header files (.h/.hpp), this will not be included in this Demo. Therefore, we encourage you to learn about and how to use header files.