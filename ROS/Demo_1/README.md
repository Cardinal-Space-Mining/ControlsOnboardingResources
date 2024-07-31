# Hello World ROS Demo
In this demo example, you will make 2 ROS nodes that will send a Hello World message between them. This will be similar to the Talk/Listen except with your OWN code.

**The `src` directory is the solution. We encourage you follow along with the steps provided**

## Setup
1. To start ROS program, create a new directory: `mkdir demo1_ws` then enter workspace with `cd demo1_ws`
2. Create a `src` directory inside `demo1_ws` using `mkdir src`
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
11. ***TODO Finish this***

### subscriber.cpp

## Run new ROS program DEMO_1