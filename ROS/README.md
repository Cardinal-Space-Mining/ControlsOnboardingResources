# ROS (Robot Operating System)
## Components in ROS
ROS is built around two main components: `Nodes` and `Services`. These components work together to create a robust system where multiple subsystems can communicate and process information efficiently and in a controlled manner.

### Nodes
In ROS, each Node is an individual program that runs on a robot. You can think of a Node as a specific subsystem, such as a motor control system, sensor input system, data analysis system, or decision-making system. Another way to think of a Node is as a worker, where each worker has a specific job, but a collection of workers together creates a product.

### Services
Services, on the other hand, are the means by which individual nodes can communicate with each other. Services use a request/response pattern. For example, in our club, the decision-making system could request new data to be analyzed to determine the best path between two points or to ascertain the robot’s location at any given point.

## Using ROS in Linux
To use ROS2 in a new terminal session, you’ll find that ros2 is not a command that you can use directly. Instead, you must run the command source /opt/ros/{distro}/setup.bash before you can run a ROS2 program, where {distro} is the name of the ROS distribution you are using (‘jazzy’ or ‘humble’).

If you’re familiar with Linux, you might know that there’s a way to run this command automatically every time a new terminal is opened. If you wish to do this, you can add this command to the .bashrc file in your home directory.

## Examples and Tutorials
In this directory will be many different tutorials and challenges to complete to learn how to use ROS and examples of how are system may work.