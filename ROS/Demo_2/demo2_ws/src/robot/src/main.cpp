#include <memory>
#include <span>
#include <utility>
#include <vector>
#include <cstdint>
#include <unistd.h>
#include <chrono>

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

class InstantiatedMotor {
    private:
        static std::unique_ptr<TalonFX>
        create_motor(const MotorInfo & motor_info, const std::string interface) {
            switch (motor_info.type) {
                case MotorInfo::MotorType::TalonFX:
                    return std::make_unique<TalonFX>(motor_info.motor_id, interface);
                default:
                    throw std::runtime_error("Unacounted Path");
            }
        }

        void on_msg(const custom_types::msg::TalonCtrl &msg) {
            switch (msg.mode) {
                case 0:
                    m_motor->SetControl(ctre::phoenix6::controls::DutyCycleOut(static_cast<float>(msg.value)));
                    break;
                default:
                    m_motor->SetControl(ctre::phoenix6::controls::DutyCycleOut(static_cast<float>(msg.value)));
                    break;
            }
        }

    public:
        custom_types::msg::TalonInfo get_info() {
            custom_types::msg::TalonInfo info;

            // info.temperature = m_motor->GetTemperature();
            info.bus_voltage = m_motor->GetSupplyVoltage().GetStatus();

            info.output_percent = m_motor->GetDutyCycle().GetStatus();
            info.output_voltage = m_motor->GetMotorVoltage().GetStatus();
            // info.output_current = m_motor->GetOutputCurrent();

            // info.position = m_motor->GetSelectedSensorPosition();
            // info.velocity = m_motor->GetSelectedSensorVelocity();

            return info;
        }

        InstantiatedMotor(const MotorInfo &motor_info,
                          const std::string &interface, rclcpp::Node &parent)
        : m_motor(InstantiatedMotor::create_motor(motor_info, interface))
        , m_ctrl_sub(parent.create_subscription<custom_types::msg::TalonCtrl>(
            std::string{motor_info.motor_name} + "_ctrl", 10,
            std::bind(&InstantiatedMotor::on_msg, this, std::placeholders::_1)))
        , m_info_pub(parent.create_publisher<custom_types::msg::TalonInfo>(
            std::string{motor_info.motor_name} + "_info", 10))
        {
        }

        std::unique_ptr<TalonFX> m_motor;
        std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonCtrl>>
            m_ctrl_sub;
        std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonInfo>> m_info_pub;
};

class Robot : public rclcpp::Node {
    private:
        void info_periodic() {
            for (auto & motor : m_motors) {
                auto info = motor.get_info();
                motor.m_info_pub->publish(info);
            }
        }

    public:
        Robot() : Node("robot")
        , info_timer(this->create_wall_timer(
            100ms, std::bind(&Robot::info_periodic, this)))
        {
            // creating the node
            static constexpr MotorInfo motors[] = {
                {"track_right", 0, MotorInfo::MotorType::TalonFX},
                {"track_left", 1, MotorInfo::MotorType::TalonFX}
            };

            for (const auto & motor : motors) {
                m_motors.emplace_back(motor, m_interface6, *this);
            }

            RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
        }
    
    private:
        const std::string m_interface6 = "can0";
    
    private:
        std::vector<InstantiatedMotor> m_motors;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
        rclcpp::TimerBase::SharedPtr info_timer;
};

int main(int argc, char ** argv) {

    // Init ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Robot>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}