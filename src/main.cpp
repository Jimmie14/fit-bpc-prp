#include <rclcpp/rclcpp.hpp>

#include "App.hpp"
#include "ExplorerEngine.hpp"
#include "FollowerEngine.hpp"
#include "ImuDriver.hpp"
#include "LidarDriver.hpp"
#include "LineEngine.hpp"
#include "MappingEngine.hpp"
#include "MotorDriver.hpp"
#include "NavigatorEngine.hpp"
#include "OdometryEngine.hpp"
#include "UserInputDriver.hpp"
#include "NavigatorGraphBuilder.hpp"

using namespace std;
using namespace Manhattan;

int main(const int argc, char* argv[])
{
    init(argc, argv);

    const auto app = make_shared<Core::App>();

    app->AddDriver<Core::ImuDriver>()->Enable();
    app->AddDriver<Core::LidarDriver>()->Enable();
    app->AddDriver<Core::MotorDriver>()->Enable();

    app->AddComponent<Core::OdometryEngine>();
    app->AddComponent<Core::LineEngine>();

    app->AddComponent<Core::UserInputDriver>();
    app->AddComponent<Core::MappingEngine>();
    app->AddComponent<Core::NavigatorEngine>();
    app->AddComponent<Core::FollowerEngine>();
    app->AddComponent<Core::ExplorerEngine>()->Enable();
    app->AddComponent<Core::NavigatorGraphBuilder>();

    app->Run();

    // Shutdown ROS 2
    shutdown();
    return 0;
}
