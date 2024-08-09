#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#define Phoenix_No_WPI //removes the WPI dependency

#include "ctre/phoenix6/TalonFX.hpp"
#include "custom_msg/msg/motor_ctrl.hpp"

// #include "ctre/Phoenix.h"

using namespace ctre::phoenix6;

using TalonFX = ctre::phoenix6::hardware::TalonFX;

class Robot : public rclcpp::Node {
    public:

    
    private:
        std::vector<std::unique_ptr<TalonFX>> m_motors;
        std::vector<
            std::shared_ptr<rclcpp::Subscription<custom_msg::msg::MotorCtrl>>>
        m_motor_subs;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
}