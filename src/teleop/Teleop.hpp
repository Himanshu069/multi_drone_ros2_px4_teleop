#pragma once

// PX4 interface Library includes
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
// TODO: Provide lookup information for the library includes

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>

class Teleop : public px4_ros2::ModeBase
{
    public:
        explicit Teleop(rclcpp::Node& node);

        // See ModeBase
        void onActivate() override;
        void onDeactivate() override;
        void updateSetpoint(float dt_s) override;

    private:

        enum class State
        {
            Home,
            Takeoff,
            Teleoop,
            Idle,
            Land
        };

        void switchToState(State state);
        std::string stateName(State state);
        
       
};