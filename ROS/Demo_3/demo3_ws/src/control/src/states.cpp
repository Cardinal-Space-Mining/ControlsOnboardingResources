#include "control/states.hpp"
#include "control/logitech_map.hpp"
#include "control/robot_constants.hpp"

#include <cmath>
#include <numbers>
#include <iostream>

MotorSettings TeleopStateMachine::update(const RobotState & robot,
                                         const sensor_msgs::msg::Joy & ctrl)
{
    if(ctrl.axes.size() < Ps4Mapping::Axes::NUM_AXES ||
       ctrl.buttons.size() < Ps4Mapping::Buttons::NUM_BUTTONS)
    {
        return MotorSettings();
    }

    if(ctrl.buttons[Ps4Mapping::Buttons::CROSS])
    {
        this->set_state(State::Normal, robot);
        std::cout << "State: Normal" << std::endl;
        return MotorSettings();
    }

    switch(this->current_state)
    {
    case State::Normal: return normal_state(robot, ctrl);

    case State::Autonomy: return autonomy_state(robot, ctrl);
    }
    // Unreachable
    throw std::runtime_error("Reached the unreachable");
}

namespace
{
std::array<double, 2> compute_track_scalars(double x, double y,
                                            double mag_deadzone)
{
    const double augmented_angle =
        std::atan2(x, y) +
        (std::numbers::pi /
         4.0); // x and y are inverted to make a CW "heading" angle
    double magnitude = std::sqrt(x * x + y * y);
    if(magnitude < mag_deadzone)
        return {0.0, 0.0};

    return {
        magnitude * std::sin(augmented_angle),
        magnitude *
            std::cos(
                augmented_angle) // this is the same as cos("raw theta" - pi/4) like from the original code
    };
}

double apply_deadband(double value, double deadband)
{
    if(std::abs(value) < deadband)
    {
        return 0.0;
    }
    return value;
}

bool double_near(double first, double second, double epsilon)
{
    return std::abs(second - first) < std::abs(epsilon);
}
} // namespace

MotorSettings
TeleopStateMachine::normal_state(const RobotState & state,
                                 const sensor_msgs::msg::Joy & ctrl)
{
    // State Transitions
    if(RobotConstants::TELEOMETRY)
    {
        if(ctrl.buttons[LogitechMapping::Buttons::START])
        {
            this->set_state(State::Autonomy, state);
            std::cout << "State: Switched to Autonomy" << std::endl;
            return MotorSettings();
        }
    }

    // Controls
    {
        NormalInfo & state = std::get<NormalInfo>(state_info);
        MotorSettings bot;

        // Speed Buttons
        {
            if(ctrl.buttons[Ps4Mapping::Buttons::CIRCLE])
            {
                state.speed_scalar = 0.3;
            }
            else if(ctrl.buttons[Ps4Mapping::Buttons::TRIANGLE])
            {
                state.speed_scalar = 0.7;
            }
            else if(ctrl.buttons[Ps4Mapping::Buttons::SQUARE])
            {
                state.speed_scalar = 1.0;
            }
        }

        // Joystick Drive
        {

            auto vars = compute_track_scalars(
                -ctrl.axes[Ps4Mapping::Axes::LEFTX],
                ctrl.axes[Ps4Mapping::Axes::LEFTY],
                
                RobotConstants::DRIVING_MAGNITUDE_DEADZONE_SCALAR);
            bot.track_right.mode = bot.track_right.PERCENT_OUTPUT;
            bot.track_right.value =
                vars[0] *
                state.speed_scalar; //* RobotConstants::TRACKS_MAX_VELO;

            bot.track_left.mode = bot.track_right.PERCENT_OUTPUT;
            bot.track_left.value =
                vars[1] *
                state.speed_scalar; //* RobotConstants::TRACKS_MAX_VELO;
        }
        
        return bot;
    }
}

MotorSettings
TeleopStateMachine::autonomy_state(const RobotState & state,
                                        const sensor_msgs::msg::Joy & ctrl)
{
        // State Transitions
    if(RobotConstants::TELEOMETRY)
    {
        if(ctrl.buttons[LogitechMapping::Buttons::START])
        {
            this->set_state(State::Normal, state);
            std::cout << "State: Switched to Normal" << std::endl;
            return MotorSettings();
        }
    }
}

void TeleopStateMachine::set_state(State state, const RobotState & robot)
{
    switch(state)
    {
    case State::Normal: state_info = NormalInfo(); break;

    case State::Autonomy: state_info = AutonomyInfo(); break;

    default: break;
    }
    this->current_state = state;
}