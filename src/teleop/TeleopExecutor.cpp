#include "TeleopExecutor.hpp"

TeleopExecutor::TeleopExecutor(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(node, Settings{}, owned_mode), _node(node)
{ }
void TeleopExecutor::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "TeleopExecutor activated");
    switchToState(State::Takeoff, px4_ros2::Result::Success);
}
void TeleopExecutor::onDeactivate(DeactivateReason reason)
{
    RCLCPP_INFO(_node.get_logger(), "TeleopExecutor deactivated");
}
void TeleopExecutor::switchToState(State state, px4_ros2::Result previous_result)
{
    _state = state;
    if (previous_result != px4_ros2::Result::Success) {
        RCLCPP_WARN(_node.get_logger(), "Switching to state %d due to previous result: %d", static_cast<int>(state), static_cast<int>(previous_result));
    };

    RCLCPP_INFO(_node.get_logger(), "Switched to state: %d", static_cast<int>(state));
    
    // Handle state-specific logic here
    switch (state) {
        case State::Takeoff:
            RCLCPP_INFO(_node.get_logger(), "Initiating takeoff...");
            takeoff([this](px4_ros2::Result result){switchToState(State::TeleOperation, result);}, 15.0f);
            break;
        case State::TeleOperation:
            scheduleMode(ownedMode().id(), [this](px4_ros2::Result result)
            {
                switchToState(State::Land, result);
            }
        );
            break;
        case State::RTL:
            rtl([this](px4_ros2::Result result) {switchToState(State::WaitUntilDisarmed, result);});
            break;
        case State::Land:
            land([this](px4_ros2::Result result) {
        switchToState(State::WaitUntilDisarmed, result);
          });
            break;
        case State::WaitUntilDisarmed:
            waitUntilDisarmed(
          [this](px4_ros2::Result result) {
            RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
          });
            break;
    }
}
using TeleopNodeWithExecutor = px4_ros2::NodeWithMode<Teleop>;


static const std::string kNodeName = "teleop_node";
static const bool kEnableDebugOutput = true;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node_with_mode = std::make_shared<TeleopNodeWithExecutor>(kNodeName, kEnableDebugOutput);

  rclcpp::spin(node_with_mode);

  rclcpp::shutdown();
  return 0;
}

