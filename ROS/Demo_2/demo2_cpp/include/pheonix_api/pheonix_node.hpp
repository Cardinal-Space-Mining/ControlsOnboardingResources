#ifndef ROS_PHOENIX_BASE
#define ROS_PHOENIX_BASE

#include "rclcpp/rclcpp.hpp"

#define Pheonix_No_WPI // removes the WPI dependencies
#include "ctre/Pheonix.h" // library containing Pheonix v6

#include <chrono> // standard cpp library for time

using namespace rclcpp;
using namespace std::chrono_literals;

namespace ros_pheonix {

    template <class MotorController, class Configuration, class ControlMode>
    class PheonixNode : public Node {

        public:
            explicit PheonixNode(const std::string& name, const NodeOptions& options = NodeOptions()) : 

    }

};

#endif ROS_PHOENIX_BASE