#include <memory>
#include <span>
#include <utility>
#include <vector>
#include <cstdint>
#include <unistd.h>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix/export.h"

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

using namespace ctre::phoenix6;
using namespace std::chrono_literals;

using TalonFX = ctre::phoenix6::hardware::TalonFX;
using std::placeholders::_1;

struct MotorInfo
{

    const char * motor_name;
    int motor_id;
    enum class MotorType
    {
        TalonFX
    };
    MotorType type;
};

class Robot : public rclcpp::Node {

    public:
        Robot() : Node("robot") {
            right_track_ctrl = this->create_subscription<custom_types::msg::TalonCtrl>(
                "right_track_ctrl", 10, std::bind(&Robot::execute_ctrl, this, _1)
            );
        }

    private:
        void execute_ctrl(const custom_types::msg::TalonCtrl::SharedPtr msg) const {
            switch ((int8_t) msg->id) {
                case 0:
                    RCLCPP_INFO(this->get_logger(), "Motor id: %d set to: %f", msg->id, msg->value);
                    m_motors[msg->id].SetControl(ctre::phoenix6::controls::DutyCycleOut(static_cast<float>(msg->value)));
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Not valid motor id: %d", msg->id);
            }
        }

        void info_periodic() {
            // for (auto & motor : m_motors) {
            //     auto info = motor.get_info();
            //     motor.m_info_pub->publish(info);
            // }
        }
    
    private:
        const std::string m_interface6 = "can0";

        static constexpr MotorInfo motors[] = {
            {"track_right", 0, MotorInfo::MotorType::TalonFX}
            // {"track_left", 1, MotorInfo::MotorType::TalonFX}
        };

        std::vector<TalonFX> m_motors = {
            {"track_right", m_interface6}
        };
    
    private:
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
        rclcpp::TimerBase::SharedPtr info_timer;

        rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr right_track_ctrl;

        
};

int main(int argc, char ** argv) {

    // Init ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Robot>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}