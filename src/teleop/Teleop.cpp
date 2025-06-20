#include "Teleop.hpp"

#incluse <px4_ros2/components/node_with_mode.hpp>

static const std::string kModeName = "Teleop";
static const bool kEnableDebug = true;

Teleop::Teleop(rclcpp::Node& node)
    : px4_ros2::ModeBase(node, kModeName)
    , _node(node)

    {}

void Teleop::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "Teleop mode activated");
    switchToState(State::Home);
}
void Teleop::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "Teleop mode deactivated");
}
void Teleop::updateSetpoint(float dt_s)
{
    // Update the setpoint based on the current state
    switch (_state) {
        case State::Home:
            // Logic for Home state
            break;
        case State::Takeoff:
            // Logic for Takeoff state
            break;
        case State::Teleoop:
            // Logic for Teleoop state
            break;
        case State::Idle:
            // Logic for Idle state
            break;
        case State::Land:
            // Logic for Land state
            break;
    }
}
std::string Teleop::stateName(State state)
{
    switch (state) {
        case State::Home: return "Home";
        case State::Takeoff: return "Takeoff";
        case State::Teleoop: return "Teleoop";
        case State::Idle: return "Idle";
        case State::Land: return "Land";
        default: return "Unknown";
    }
}

void Teleop::switchToState(State state)
{
    _state = state;
    RCLCPP_INFO(_node.get_logger(), "Switched to state: %s", stateName(state).c_str());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<Teleop>>(kNodeName, kEnableDebug));
    rclcpp::shutdown();
    return 0;
}