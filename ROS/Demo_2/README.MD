# Fuzzyball - Teleoperated

**Please READ this and code**, ask questions

The goal for this Demo is to create a ROS program using the Phoenix 6 API such that using a joystick controller you can drive the test chassis around. Also in this demo, you will/should learn how to collect and log data throughout your run. This is crucial for our club, during our runs and tests we collect data regarding current, temperatures, etc... that we can analyze to determine if there are any issues and how we could fix them. In the case of a motor going into thermal shutdown we can analyze how this happened and normally current is involved.

In this demo, you will create a ROS program to control the test chassis/robot with a controller. In standard practice for our club, we will use a Logitch controller. This form of control is called Teleoperated or Teleop for short. In the Lunabotics competition we are allowed to control our robot with this system. Although we will lose points for this.

This will be a very basic implementation project. In our development, some practices may vary including how ROS messages may be sent.

**PLEASE LOOK OVER THIS CODE**, we encourage you try to create/build this code yourself. You do not need to recreate everything. This `README` will go over elements of this project. 

Extra credit (experience/troubleshooting): Report back the status of motors (i.e. temps, current, voltage) of motors and print out in the control mode. See the Phoenix6 API for more info. This would be done by creating a publisher node on the robot with a subscriber node in the control. This would also be on a timer, examples of this would be the heartbeat (see code).

## ROS PKG: custom_types
This package serves 1 purpose, to give us a custom message type to send data from `Control` and `Robot` nodes. There is no `c++` code in this package, instead there are 2 `.msg` files containing custom message formats. When code is build/compiled and ran the `c++` files are automatically generated for use within program. This is done with following lines in `CMakeLists.txt` and `package.xml`:

CMakeLists.txt:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TalonCtrl.msg"
  "msg/TalonInfo.msg"
)
```
package.xml
```xml
<buildtool_depend>residl_default_generators</buildtool_depend>

<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## ROS PKG: control
The control node is where all controller input is handled as well as publishing motor control messages. This node uses many different Class objects. If you have questions about any specifics regarding each class please ask.

There are some files included that contain control functions from last year with the intent of using again this year. The files included are: `states.hpp`/`ps4_states.hpp` and `robot_constants.hpp`. The controller mapping files: `logitech_map.hpp` and `ps4_map.hpp` only contain the constants regarding controller inputs (used in `states.hpp`).

Note: while developing this project I used a PS4 controller, when using the Club Logitech controller comment out `#include "control/ps4_states.hpp"` and instead use `#include "control/states.hpp"`

`states.hpp/states.cpp` files contain all code taking in controller input from the `joy_node` and translate code into motor controls. Instead of waiting for any input from joystick, control commands are being sent continuously every 100 milliseconds, including calculations of new motor settings via the `RobotTeleopInterface` classes `update_motors()`.

## ROS PKG: robot
This node is pretty similar compared to the control node. We could have handled joystick input here to reduce network usage, but due to limited computing power on robot we prefer to handle it locally. 

There are no header files in this node, if you wanted to practice using header files, you could change this node to include them.

For each motor, there is a subscription service. There also is a timer and a heartbeat subsciption. This subscription is needed to keep all motors active since we will not be using the FRC driver station. Instead we use the `unmanaged` library in `phoenix6/phoenix5` api. As you can see, the timer calls a function `info_periodic`, if you wish to gain experience you would work here to send motor status information with this function using the `custom_types::msg::TalonInfo` message type.

## Running Program
Running program can be done in many ways. Only 1 package (robot) could be built and ran on robot, or you can run and build all on both your computer and on the robot for simplicity.

To run code on the robot, you will need to `ssh` into the robot (there may be more depending on CAN device in use).

1. From the `demo2_ws/src` directory run the command `./build.sh` (if you cant run because of permissions, run command `sudo chmod 700 build.sh`)
2. Once finished building, you will **need 2** terminals on your machine and **1** running on the robot (we will help with this). 
3. On your machine run `source install/setup.bash` from the `src` directory in both terminals.
4. In one terminal run `ros2 run control main` and in the other run `ros2 run joy joy_node`. 
5. On the robot, run `ros2 run robot main` to start ROS on the robot.
6. Assuming all nodes are running correctly you should be able to control robot using the left joystick.