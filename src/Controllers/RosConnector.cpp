#include "../../include/Common/RosConnector.hpp"

#include "App.hpp"

namespace Manhattan::Core {
RosConnector::RosConnector(const App& app)
    : _app(app)
    , _node(app.GetNode())
{
}

void RosConnector::Enable()
{
    if (_enabled)
        return;
    _enabled = true;

    OnEnable();
}

void RosConnector::Disable()
{
    if (!_enabled)
        return;
    _enabled = false;

    OnDisable();
}
} // namespace Manhattan::Core
