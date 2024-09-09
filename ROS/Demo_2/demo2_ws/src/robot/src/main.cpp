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
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

using namespace ctre::phoenix6;
using namespace std::chrono_literals;

using TalonFX = ctre::phoenix6::hardware::TalonFX;
using std::placeholders::_1;

namespace constants
{
static const std::string INTERFACE = "can0";
} // namespace constants

class Robot : public rclcpp::Node {

    public:
        Robot() : Node("robot")
        , heartbeat_sub(this->create_subscription<std_msgs::msg::Int32>(
            "heartbeat", 10, [](const std_msgs::msg::Int32 &msg)
            { ctre::phoenix::unmanaged::FeedEnable(msg.data); })) 
        , info_timer(
            this->create_wall_timer(100ms, [this]() { this->info_periodic(); }))

        , track_right_ctrl(this->create_subscription<custom_types::msg::TalonCtrl>(
            "track_right_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg) 
            { execute_ctrl(this->track_right, msg); }))
            
        , track_left_ctrl(this->create_subscription<custom_types::msg::TalonCtrl>(
            "track_left_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg)
            { execute_ctrl(this->track_left, msg); }))
        {

            std::array<std::reference_wrapper<TalonFX>, 2> motors = {
                {track_right, track_left}};

            for (auto &motor : motors) {
                config_talonfx(motor);
            }

            RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
        }

    private:

        void config_talonfx(TalonFX &motor) {
            configs::TalonFXConfiguration generic_config{};

            generic_config.Slot0.kP = 0.0;
            generic_config.Slot0.kI = 0.0;
            generic_config.Slot0.kD = 0.0;
            generic_config.Slot0.kV = 0.0;
            generic_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

            generic_config.CurrentLimits.StatorCurrentLimitEnable = false;

            motor.GetConfigurator().Apply(generic_config);
        }

        void execute_ctrl(TalonFX &motor, const custom_types::msg::TalonCtrl &msg) {
            motor.SetControl(controls::DutyCycleOut(msg.value));
        }

        void info_periodic() {
            // for (auto & motor : m_motors) {
            //     auto info = motor.get_info();
            //     motor.m_info_pub->publish(info);
            // }
        }
    
    private:
        TalonFX track_right{0, constants::INTERFACE};
        TalonFX track_left{1, constants::INTERFACE};  
    
    private:
        rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr track_right_ctrl;
        rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr track_left_ctrl;

    private:
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
        rclcpp::TimerBase::SharedPtr info_timer;
        
};

int main(int argc, char ** argv) {

    // Init ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    // ctre::phoenix::unmanaged::FeedEnable(100);
    ctre::phoenix::unmanaged::LoadPhoenix();

    auto node = std::make_shared<Robot>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}