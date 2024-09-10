#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <SDL2/SDL.h>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "control/logitech_map.hpp"
#include "control/states.hpp"

// #include "control/ps4_map.hpp"
// #include "control/ps4_states.hpp"

#include "custom_types/msg/talon_ctrl.hpp"

enum class ControlMode {
    DISABLED,
    TELEOP,
    AUTONOMOUS
};

/**
 * TODO
 * Implement states.hpp
 */

using namespace std::chrono_literals;

class RobotTeleopInterface {
private:
    void update_motors() {
        MotorSettings motor_settings =
            this->teleop_state.update(this->robot_state, this->joy);
        track_right_ctrl->publish(motor_settings.track_right);
        track_left_ctrl->publish(motor_settings.track_left);
    }

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> talon_ctrl_pub(rclcpp::Node &parent, const std::string &name) {
        return parent.create_publisher<custom_types::msg::TalonCtrl>(name, 10);
    }

public:
    RobotTeleopInterface(rclcpp::Node &parent) : 
        joy_sub(parent.create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, [this](const sensor_msgs::msg::Joy &joy) { this->joy = joy; }))
        , track_right_ctrl(talon_ctrl_pub(parent, "track_right_ctrl"))
        , track_left_ctrl(talon_ctrl_pub(parent, "track_left_ctrl"))
        , teleop_update_timer(
            parent.create_wall_timer(100ms, [this]() { this->update_motors(); }))
        {
        }

private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;
    
    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> track_right_ctrl;
    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> track_left_ctrl;

private:
    sensor_msgs::msg::Joy joy;
    RobotState robot_state;

private:
    rclcpp::TimerBase::SharedPtr teleop_update_timer;
    TeleopStateMachine teleop_state;
};

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller_node")
    , heartbeat(create_publisher<std_msgs::msg::Int32>("heartbeat", 10))
    , heartbeat_timer(this->create_wall_timer(100ms,
        [this]()
        {
            if(this->bot_enabled) {
                std_msgs::msg::Int32 msg;
                msg.data =
                    ENABLE_TIME.count();
                this->heartbeat->publish(msg);
            }
        }))
    , teleop_interface(*this)
    {
    }

    ~Controller()
    {
    }

private:
    RobotTeleopInterface teleop_interface;

    sensor_msgs::msg::Joy joy;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr heartbeat;

private:

    bool bot_enabled = true;
    static constexpr auto ENABLE_TIME = 250ms;

    rclcpp::TimerBase::SharedPtr timer_;
    ControlMode current_state = ControlMode::DISABLED;

};


int main(int argc, char * argv[])
{
    // Load ROS2
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
