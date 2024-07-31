# Hello World ROS Demo
In this demo example, you will make 2 ROS nodes that will send a Hello World message between them. This will be similar to the Talk/Listen except with your OWN code.

You will need 2 Ubuntu terminals open to complete this Demo. Please make sure all 

**The `solution` directory contains the solutions for all edited files. We encourage you follow along with the steps provided**

Inspiration and development of this demo: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

## Setup
1. To start ROS program, create a new directory: `mkdir demo1_ws` then enter workspace with `cd demo1_ws`
2. Create a `src` directory inside `demo1_ws` using `mkdir src`. Then using `cd` move into the src directory
3. To create the ROS package, use `ros2 pkg create --build-type ament_cmake cpp_hello_world`. This will create all folders and files required automatically for this project.
4. Using `cd` navigate to `demo1_ws/src/cpp_hello_world/src`. This is where any created cpp files belong. When code is compiled, the compiler will look in this src directory.
5. Either using VS Code or the terminal create 2 files: `publisher.cpp` and `subscriber.cpp`. These 2 files are where your nodes will be.

### publisher.cpp
1. First, we will work on `publisher.cpp`. Add the following libraries to `publisher.cpp`
```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

/* Common parts in ROS2 */
#include "rclcpp/rclcpp.hpp"
/* Built in message type */
#include "std_msgs/msg/string.hpp"
```
The first 4 libraries are common standard c++ libraries. "rclcpp/rclcpp.hpp" is required as a header file that is used as a template for our publisher node.

2. To reduce repetitive type-casting we can use (note: **DO NOT EVER** use `using namespace std`): 
```cpp
/* Practice to avoid using std::chrono_literals constantly */
using namespace std::chrono_literals;
```

3. Create a `class MinPublisher` using inheritance from rclcpp::Node
```cpp
class MinPublisher : public rclcpp::Node {
    // Class items private/public/protected will go here
};
```
4. Create the `public` function(s), in this case is the _constructor_
```cpp
    public:
        /* Constructor for class object */
        MinPublisher() : Node("pub"), count_(0) {
            pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer_ = this->create_wall_timer(
                    500ms, std::bind(&MinPublisher::timer_callback, this)
            );
        } 
```
5. Then create the `private` function(s), and data types.
```cpp
    private:
        /* This is where the message is sent from. all messages sent are also logged in console*/
        void timer_callback() {
            auto message = std_msgs::msg::String();
            message.data = "Hello World!" + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            pub_->publish(message);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        size_t count_;
```
6. Finally, add the main function for this class
```cpp
int main(int argc, char* argv[]) {
    // initializes ROS2
    rclcpp::init(argc, argv);
    // starts processing data from node
    rclcpp::spin(std::make_shared<MinPublisher>());
    // stops ROS2
    rclcpp::shutdown();
    return 0;
}
```
7. Next, the dependencies need to be added to `package.xml`, this file can be found in the `cpp_hello_world` directory.
8. Add the following dependencies after `<buildtool_depend>...</...>`
```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```
9. Next, we need to update `CMAKELists.txt` to create a ROS node. Below `find_package(ament_cmake REQUIRED)` add these 2 dependencies:
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```
10. Then add the executable `talker`. If this is **not added**, it **cannot** be ran using `ros2 run`
```cmake
add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```
11. In order for our talker node to be usable, we need to install it for when we use `ros2 run`. If we do not do this, `ros2 run` will not be able to find the executable. Below the add executable (step 10) add the following lines.
```cmake
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

### subscriber.cpp
1. Add the following libraries to `subscriber.cpp`
```cpp
#include <memory>

/* Common parts in ROS2 */
#include "rclcpp/rclcpp.hpp"
/* Built in message type */
#include "std_msgs/msg/string.hpp"
```
2. To reduce repetitive type-casting we can use (note: **DO NOT EVER** use `using namespace std`): 
```cpp
using std::placeholders::_1;
```
3. Create the `MinSubscriber` class using inheritance from `rclcpp::Node`
```cpp
class MinSubscriber : public rclcpp::Node {
    public:
    // public functions (constructor)/data types
    private:
    // private functions/data types
}
```
4. In the `public` section of the class, create the contstructor class:
```cpp
/* Constructor for subscriber node. This will create the subscriber object */
MinSubscriber() : Node("sub") {
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinSubscriber::topic_callback, this, _1)
    );
}
```
5. In the `private` section, add the `topic_callback` and pointer `sub_`:
```cpp
/* Receives the message data that is published in topic, then writes to console */
void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}
/* field declaration */
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
```
6. Add the `main` function for this node:
```cpp
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```
7. In order for this node to be executable, and can be find by the `ros2 run` command, we need to add executable and `install(TARGETS...)`. In the `CMakeLists.txt` add the following lines beneath the publisher executable/install:
```cmake
add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
  
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

## Run new ROS program DEMO_1
1. In a Ubuntu terminal from the root of your workspace `demo1_ws`, run the command `rosdep install --from-path src` to install required ROS packages/dependencies for your program.
2. To build your program, simply use the command `colcon build` from again the root directory `demo1_ws`.
3. In both terminals, you will need to source project files to use `ros2 run` command and find the correct files. Run the command `source install/setup.bash`
4. In 1 terminal run: `ros2 run cpp_hello_world talker` to start the talking node.
5. In the second terminal run: `ros2 run cpp_hello_world listener` to start the listening node. It should start receiving messages from the talker node.
6. (Optional) If you wish to view a diagram of the nodes running, open a third terminal and run: `rqt_graph`
