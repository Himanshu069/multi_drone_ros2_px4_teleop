#pragma once

#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

#include "Teleop.hpp"

class TeleopExecutor : public px4_ros2::ModeExecutorBase
{
    public:
        TeleopExecutor(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode);

        void onActivate() override;
        void onDeactivate(DeactivateReason reason) override;

    private:
        enum class State
        {
            Takeoff,
            TeleOperation,
            RTL,
            Land,
            WaitUntilDisarmed
        };
    void switchToState(State state, px4_ros2::Result previous_result);
    rclcpp::Node & _node;
    State _state;



};