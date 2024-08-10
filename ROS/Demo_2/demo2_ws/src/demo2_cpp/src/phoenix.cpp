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

            //uncomment if can device installed
            motors.emplace_back(std::make_unique<TalonFX>(0, CANBUS)); //right motor
            motors.emplace_back(std::make_unique<TalonFX>(1, CANBUS)); //left motor

            rec_command = this->create_subscription<custom_msg::msg::MotorCtrl>(
                "topic", 10, std::bind(&Robot::topic_callback, this, _1)
            );
        }

    private:
    
        //uncomment if can device installed

        // Name/serialnumber/SocketCAN interface of Canivore/Canible
        // using "*" to select any, dont use this
        static constexpr char const *CANBUS = "can0"; 

        std::vector<std::unique_ptr<TalonFX>> motors;

        void topic_callback(const custom_msg::msg::MotorCtrl::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "Motor '%d' into Mode '%d' with '%f' output", msg->id, msg->mode, msg->value);

            // int8_t id = (int8_t) msg->id;
            // int8_t mode = (int8_t) msg->mode;
            // auto output = msg->value;

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