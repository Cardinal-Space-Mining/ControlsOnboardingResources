#ifndef MISSION_CTRL_STATES_HPP_6_27_2024
#define MISSION_CTRL_STATES_HPP_6_27_2024

#include <variant>
#include <limits>

#include "sensor_msgs/msg/joy.hpp"

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

struct MotorSettings
{
    custom_types::msg::TalonCtrl track_right;
    custom_types::msg::TalonCtrl track_left;
};

struct RobotState
{
    custom_types::msg::TalonInfo track_right;
    custom_types::msg::TalonInfo track_left;
};

class TeleopStateMachine
{
public:
    TeleopStateMachine()
    : state_info{NormalInfo()}
    {
        RobotState state;
        set_state(State::Normal, state);
    }

public:
    MotorSettings update(const RobotState & robot,
                         const sensor_msgs::msg::Joy & ctrl);

private:
    MotorSettings normal_state(const RobotState & robot,
                               const sensor_msgs::msg::Joy & ctrl);

private:
    enum class Lifecycle
    {
        Start,
        Norm,
        End
    };

    struct NormalInfo
    {
        double speed_scalar = 1.0;
    };

    struct AutonomyInfo
    {
        Lifecycle lifecycle = Lifecycle::Start;
        double track_scalar = 0.5;
    };

    enum class State
    {
        Normal,
        Autonomy
    };

private:
    void set_state(State state, const RobotState & robot);

    State current_state;
    std::variant<NormalInfo, AutonomyInfo> state_info;
};

#endif