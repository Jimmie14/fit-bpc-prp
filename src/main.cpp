#include <rclcpp/rclcpp.hpp>

#include "../include/Controllers/LineController.hpp"
#include "../include/Controllers/MotorDriver.hpp"
#include "App.hpp"
#include "ExplorerController.hpp"
#include "FollowerController.hpp"
#include "ImuDriver.hpp"
#include "LidarDriver.hpp"
#include "MappingEngine.hpp"
#include "NavigatorController.hpp"
#include "OdometryEngine.hpp"
#include "UserInputController.hpp"

using namespace std;
using namespace Manhattan;

int main(const int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    const auto app = make_shared<Core::App>();

    app->AddDriver<Core::ImuDriver>()->Enable();
    app->AddDriver<Core::LidarDriver>()->Enable();
    app->AddDriver<Core::MotorDriver>()->Enable();

    app->AddController<Core::OdometryEngine>();
    app->AddController<Core::LineController>();

    app->AddController<Core::UserInputController>();
    app->AddController<Core::MappingEngine>();
    app->AddController<Core::NavigatorController>();
    app->AddController<Core::FollowerController>();
    app->AddController<Core::ExplorerController>();

    // app->GetController<Core::FollowerController>()->Enable();
    // app->GetController<Core::ExplorerController>()->Enable();

    app->Run();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
