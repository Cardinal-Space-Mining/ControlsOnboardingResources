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

class Robot : public rclcpp::Node {
    public:
        template <size_t V>
        Robot(std::span<const std::pair<const char *, int>> motors) : rclcpp::Node("robot") {
            for (const auto & motor : motors) {
                auto motor_ptr = std::make_unique<TalonFX>(motor.second);
                m_motor_subs.emplace_back(this->create_subscription<custom_msg::msg::MotorCtrl>(
                    motor.first, 10,
                    [&motor_ptr](custom_msg::msg::MotorCtrl msg) {
                        motor_ptr->SetControl(
                            msg.value
                        )
                    }
                ));
            }
        }

    private:
        std::vector<std::unique_ptr<TalonFX>> m_motors;
        std::vector<
            std::shared_ptr<rclcpp::Subscription<custom_msg::msg::MotorCtrl>>>
            m_motor_subs;
};



int main(int argc, char** argv) {

    // Init ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    // Set the can interface for pheonix5
    const char* CANBUS = "can0";

    static constexpr TalonFX motors[] = {
        track_right{0, CANBUS},
        track_left{1, CANBUS}
    }

    auto node = std::make_shared<Robot>(std::span{std::begin(motors), std::end(motors)});

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}