#include "DwppNavigatorEngine.hpp"

#include "App.hpp"
#include "Messages/RobotMode.hpp"
#include "Messages/Nav.hpp"

namespace Manhattan::Core {

DwppNavigatorEngine::DwppNavigatorEngine(const App& app)
    : RosEngine(app, "navigator")
{
    _app.Events->Subscribe<Messages::RobotEnvironmentChangeEvent>([this](const auto& _) {

    });

    _app.Events->Subscribe<Messages::RobotFollowPathEvent>([this](const auto& event) {

    });
}

}
