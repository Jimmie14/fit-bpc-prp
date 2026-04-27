#include <rclcpp/rclcpp.hpp>

#include "App.hpp"
#include "ArucoDetectionEngine.hpp"
#include "ExplorerEngine.hpp"
#include "FollowerEngine.hpp"
#include "ImuDriver.hpp"
#include "LidarDriver.hpp"
#include "LineEngine.hpp"
#include "MappingEngine.hpp"
#include "MotorDriver.hpp"
#include "NavigatorEngine.hpp"
#include "NavigatorGraphBuilder.hpp"
#include "OdometryEngine.hpp"
#include "UserInputDriver.hpp"

using namespace std;
using namespace Manhattan;

int main(const int argc, char* argv[])
{
    init(argc, argv);

    const auto app = make_shared<Core::App>();

    app->AddDriver<Core::ImuDriver>()->Enable();
    app->AddDriver<Core::LidarDriver>()->Enable();
    app->AddDriver<Core::MotorDriver>()->Enable();

    app->AddEngine<Core::OdometryEngine>();
    app->AddEngine<Core::LineEngine>();

    app->AddEngine<Core::ArucoDetectionEngine>()->Enable();

    app->AddDriver<Core::UserInputDriver>();

    app->AddEngine<Core::MappingEngine>();
    app->AddEngine<Core::NavigatorEngine>();
    app->AddEngine<Core::NavigatorGraphBuilder>();
    app->AddEngine<Core::FollowerEngine>()->Enable();
    app->AddEngine<Core::ExplorerEngine>();

    app->Run();

    // Shutdown ROS 2
    shutdown();
    return 0;
}
