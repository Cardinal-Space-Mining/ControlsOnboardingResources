#include <algorithm>
#include <cstdlib>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <SDL2/SDL.h>

#include "control/logitech_map.hpp"
#include "custom_types/msg/talon_ctrl.hpp"

enum class ControlMode {
    DISABLED,
    TELEOP,
    AUTONOMOUS
};

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller_node")
    {
        // Initialize SDL
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize SDL: %s", SDL_GetError());
            rclcpp::shutdown();
        }

        if (SDL_NumJoysticks() < 1) {
            RCLCPP_ERROR(this->get_logger(), "No joysticks connected!");
            rclcpp::shutdown();
        }

        joy = SDL_GameControllerOpen(0);
        if (joy == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open joystick: %s", SDL_GetError());
            rclcpp::shutdown();
        }

        right_track_ctrl = this->create_publisher<custom_types::msg::TalonCtrl>("right_track_ctrl", 10);

        // Create a timer to poll SDL events
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Controller::poll_events, this));

        RCLCPP_INFO(this->get_logger(), "Joystick found and waiting for input");
        RCLCPP_INFO(this->get_logger(), "Current Mode: DISABLED");
    }

    ~Controller()
    {
        SDL_GameControllerClose(joy);
        SDL_Quit();
    }

private:
    void poll_events()
    {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_CONTROLLERAXISMOTION && current_state == ControlMode::TELEOP) {
                if (event.caxis.which == 0) { // First joystick

                    int x_axis = SDL_GameControllerGetAxis(joy, SDL_CONTROLLER_AXIS_LEFTX);
                    int y_axis = SDL_GameControllerGetAxis(joy, SDL_CONTROLLER_AXIS_LEFTY);

                    // Apply dead zone
                    if (abs(x_axis) < DEAD_ZONE) x_axis = 0;
                    if (abs(y_axis) < DEAD_ZONE) y_axis = 0;

                    if (x_axis == 0 && y_axis == 0) {
                        continue;
                    }

                    float left_motor_speed = motor_percent(y_axis + x_axis);
                    float right_motor_speed = motor_percent(y_axis - x_axis);

                    RCLCPP_INFO(this->get_logger(), "Left: %f \t Right: %f", left_motor_speed, right_motor_speed);
                
                    auto msg = custom_types::msg::TalonCtrl();
                    msg.id = 0;
                    msg.mode = 0;
                    msg.value = right_motor_speed;
                    right_track_ctrl->publish(msg);
                }
            } else if (event.type == SDL_CONTROLLERBUTTONDOWN) {
                // if (event.cbutton.which == 0) { // First joystick
                    if (event.cbutton.button == SDL_CONTROLLER_BUTTON_RIGHTSHOULDER && current_state == ControlMode::DISABLED) {
                        RCLCPP_INFO(this->get_logger(), "Mode changed to: TELEOP");
                        current_state = ControlMode::TELEOP;
                    } else if ((event.cbutton.button == SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) && current_state != ControlMode::DISABLED) {
                        RCLCPP_INFO(this->get_logger(), "Mode changed to: DISABLED");
                        current_state = ControlMode::DISABLED;
                    } else if (event.cbutton.button == SDL_CONTROLLER_BUTTON_LEFTSHOULDER && current_state == ControlMode::DISABLED) {
                        RCLCPP_INFO(this->get_logger(), "Mode changed to: AUTONOMOUS");
                        current_state = ControlMode::AUTONOMOUS;
                    } else if (event.cbutton.button == SDL_CONTROLLER_BUTTON_LEFTSHOULDER && current_state == ControlMode::AUTONOMOUS) {
                        RCLCPP_INFO(this->get_logger(), "Mode changed to: DISABLED");
                        current_state = ControlMode::DISABLED;
                    }
                // }
            }
        }
    }

    float motor_percent(int value) {

        float percent = (value * 100) / 32767;

        if (percent < 0) {
            return std::max(percent, (float) -100.0);
        } else {
            return std::min(percent, (float) 100.0);
        }
    }

    SDL_GameController* joy;
    rclcpp::TimerBase::SharedPtr timer_;
    ControlMode current_state = ControlMode::DISABLED;

    const int DEAD_ZONE = 8000;

private:
    rclcpp::Publisher<custom_types::msg::TalonCtrl>::SharedPtr right_track_ctrl;

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