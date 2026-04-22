#include "RosComponent.hpp"

#include "App.hpp"

namespace Manhattan::Core {
RosComponent::RosComponent(const App& app, const std::string& nodeName)
    : Node(nodeName)
    , _app(app)
{

}

void RosComponent::Enable()
{
    if (_enabled)
        return;
    _enabled = true;

    OnEnable();
}

void RosComponent::Disable()
{
    if (!_enabled)
        return;
    _enabled = false;

    OnDisable();
}
} // namespace Manhattan::Core
