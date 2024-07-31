/* Standard C++ headers */
#include <memory>

/* Common parts in ROS2 */
#include "rclcpp/rclcpp.hpp"

/* Built in message type */
#include "std_msgs/msg/string.hpp"

using  std::placeholders::_1;

/* This is a class that inherits functionality from the ROS2 Node class*/
class MinSubscriber : public rclcpp::Node {
    public:
      /* Constructor for subscriber node. This will create the subscriber object */
      MinSubscriber() : Node("sub") {
        sub_ = this->create_subscription<std_msgs::msg::String>(
          "topic", 10, std::bind(&MinSubscriber::topic_callback, this, _1)
        );
      }
    
    private:
      /* Receives the message data that is published in topic, then writes to console */
      void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      }
      /* field declaration */
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinSubscriber>());
  rclcpp::shutdown();
  return 0;
}
