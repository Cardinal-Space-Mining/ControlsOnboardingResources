# Setting up the Working Environment
Our robot runs on Ubuntu Linux 22.04 or newer. It is recommended that you have an understanding of how Linux works and how to operate within Linux. In the tutorials and challenges, there will be opportunities to learn and practice using Linux.

As most current members use Windows as their main operating system, we often use WSL2 or Windows Subsystem for Linux 2 as our primary terminal and interface to run/build code locally.

## VS Code
On the controls team, we primarily use VS Code as our IDE/Code editor. The versatility it provides is the main reason it is used. We use a specific version of VS Code because there is a custom version that allows for easier project development with features specific to our motor controllers. We use both Phoenix5 and Phoenix6 APIs in our project.

We use FRC (First Robotics Competition) software and hardware on our rover each year. In order to work on and create software for our robot, we use the WPILib version of VS Code.

To install WPILib VS Code, go to: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html, and use the current installer.

## ROS (Robot Operating System)
While we use FRC and WPILib to control our motors, we need something to control all subsystems. To do this, we use ROS or Robot Operating System. This is an industry-standard system to control different services with great functionality and support.

To keep development similar to how it will be run on the robot, we will be setting up ROS in WSL2.

During the 23-24 season, we used ROS2 Humble since it will receive long-term support (until 2027), but we will most likely be changing to ROS2 Jazzy (note: ROS2 Jazzy is for Ubuntu 24.04 version or newer).

To install ROS2 Jazzy, follow the instructions found at: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html. If you wish, you can also do the examples at the end of the installation instructions.

### To use and create ROS projects you will need `colcon`
To install `colcon`, open a Ubuntu Terminal and run `sudo apt install python3-colcon-common-extensions`. You will need to also enter `y` to confirm install.

### Install/find ROS dependencies (rosdep)
While building ROS projects, certain dependencies/packages may need to be installed in your project. To do this, you will need to install `rosdep`.

1. In a Ubuntu terminal run: `sudo apt-get install python3-rosdep`
2. Initialize & update rosdep: `sudo rosdep init` and `rosdep update`
3. Since we will be using ROS2 Jazzy, a simple way for rosdep to function without repetitively running the same command, we will add `export ROS_DISTRO=jazzy` to our .bashrc file. Open using nano or vim the .bashrc file `nano ~/.bashrc` or `vim ~/.bashrc` and append `export ROS_DISTRO=jazzy` to the end of the file. Then save and exit file. Now, anytime you open a new Ubuntu terminal it will be configured to run ROS2 Jazzy.


## Sources (not in order/complete)
rosdep: https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#rosdep-installation

ROS2 jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html
