#include "Common/RosComponent.hpp"

#include "App.hpp"

namespace Manhattan::core {
RosComponent::RosComponent(const App& app, const std::string& nodeName)
    : Node(nodeName, NodeOptions().use_intra_process_comms(true))
    , _app(app)
{
}

void RosComponent::Enable()
{
    if (_enabled) return;

    _enabled = true;

    OnEnable();

    RCLCPP_INFO(this->get_logger(), "%s enabled", get_name());
}

void RosComponent::Disable()
{
    if (!_enabled)
        return;
    _enabled = false;

    OnDisable();

    RCLCPP_INFO(this->get_logger(), "%s disabled", get_name());
}
} // namespace Manhattan::Core
