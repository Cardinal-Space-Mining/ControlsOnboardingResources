
#include <iostream>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp> 

#include "custom_msg/msg/motor_ctrl.hpp"
#include "demo2_cpp/LogitechConstants.hpp"

using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
    public:
        /* Constructor for Controller Class */
        Controller() : Node("controller") {
            send_command = this->create_publisher<custom_msg::msg::MotorCtrl>("topic", 10);
            timer = this->create_wall_timer(
                1000ms, std::bind(&Controller::timer_callback, this)
            );
        }

    private:
        void timer_callback() {
            auto message = custom_msg::msg::MotorCtrl();
            message.mode = 0;
            RCLCPP_INFO(this->get_logger(), "Published: '%d'", message.mode);
            send_command->publish(message);
        }

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<custom_msg::msg::MotorCtrl>::SharedPtr send_command;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Controller>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    node = nullptr;
    
    return 0;
}