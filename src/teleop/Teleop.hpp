#pragma once

// PX4 Interface Library
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/manual_control_input.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/utils/geometry.hpp>

// ROS 2 Core
#include <rclcpp/rclcpp.hpp>

// C++ Std
#include <cmath> // for M_PI
#include <Eigen/Eigen>

class Teleop : public px4_ros2::ModeBase
{
    public:
        explicit Teleop(rclcpp::Node& node);

        // See ModeBase
        void onActivate() override;
        void onDeactivate() override;
        void updateSetpoint(float dt_s) override;

    private:
        std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
        std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
        std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;
        rclcpp::Node & _node;
        float _yaw{0.f};      
       
};