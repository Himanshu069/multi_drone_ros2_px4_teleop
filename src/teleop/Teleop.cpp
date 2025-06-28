#include "Teleop.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

static const std::string kModeName = "Teleoperation";
static const bool kEnableDebug = true;

Teleop::Teleop(rclcpp::Node& node)
    : px4_ros2::ModeBase(node, kModeName)
    , _node(node)

    {
        _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
        _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
        _attitude_setpoint = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
    }

void Teleop::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "Teleop mode activated");
}
void Teleop::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "Teleop mode deactivated");
}
void Teleop::updateSetpoint(float dt_s)
{
    if (!_manual_control_input->isValid()) {
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000, "Manual control input not valid.");
        return;
    }

    const float roll = _manual_control_input->roll();
    const float pitch = _manual_control_input->pitch();
    const float yaw = _manual_control_input->yaw();
    const float throttle = _manual_control_input->throttle();

    const float threshold = 0.9f;
    const bool want_rates = fabsf(roll) > threshold || fabsf(pitch) > threshold;

    const float yaw_rate = px4_ros2::degToRad(yaw * 120.f);

    const Eigen::Vector3f thrust_sp{0.f, 0.f, -throttle};

    if (want_rates) {
        const Eigen::Vector3f rates_sp{
            px4_ros2::degToRad(roll * 500.f),
            px4_ros2::degToRad(-pitch * 500.f),
            yaw_rate
        };
        _rates_setpoint->update(rates_sp, thrust_sp);
    } else {
        _yaw += yaw_rate * dt_s;

        const Eigen::Quaternionf qd = px4_ros2::eulerRpyToQuaternion(
            px4_ros2::degToRad(roll * 55.f),
            px4_ros2::degToRad(-pitch * 55.f),
            _yaw
        );
        _attitude_setpoint->update(qd, thrust_sp, yaw_rate);
    }

}