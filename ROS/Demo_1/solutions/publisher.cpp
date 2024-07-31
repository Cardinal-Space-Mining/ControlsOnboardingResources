/* Standard C++ headers */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

/* Common parts in ROS2 */
#include "rclcpp/rclcpp.hpp"

/* Built in message type */
#include "std_msgs/msg/string.hpp"

/* Practice to avoid using std::chrono_literals constantly */
using namespace std::chrono_literals;

/* This is a class that inherits functionality from the ROS2 Node class*/
class MinPublisher : public rclcpp::Node {
    public:
        /* Constructor for class object */
        MinPublisher() : Node("pub"), count_(0) {
            pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer_ = this->create_wall_timer(
                    500ms, std::bind(&MinPublisher::timer_callback, this)
            );
        }
    
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
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinPublisher>());
    rclcpp::shutdown();
    return 0;
}