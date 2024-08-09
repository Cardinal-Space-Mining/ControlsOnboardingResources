#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#define Phoenix_No_WPI //removes the WPI dependency

#include "ctre/phoenix6/TalonFX.hpp"
#include "custom_msg/msg/motor_ctrl.hpp"

using namespace ctre::phoenix6;

using TalonFX = ctre::phoenix6::hardware::TalonFX;
using std::placeholders::_1;

class Robot : public rclcpp::Node {
    public:
        Robot() : Node("robot") {
            rec_command = this->create_subscription<custom_msg::msg::MotorCtrl>(
                "topic", 10, std::bind(&Robot::topic_callback, this, _1)
            );
        }

    private:
        // Name/serialnumber/SocketCAN interface of Canivore/Canible
        // using "*" to select any, dont use this
        static constexpr char const *CANBUS = "*";

        TalonFX left_track {0, CANBUS};
        TalonFX right_track {1, CANBUS}; 

        void topic_callback(const custom_msg::msg::MotorCtrl::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->mode);
        }    

        rclcpp::Subscription<custom_msg::msg::MotorCtrl>::SharedPtr rec_command;

        

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Robot>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}