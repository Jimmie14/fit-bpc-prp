#include <rclcpp/rclcpp.hpp>

#include "App.h"
#include "Controllers/Node/LineController.hpp"
#include "ExplorerController.hpp"
#include "FollowerController.hpp"
#include "ImuComponent.hpp"
#include "LidarController.hpp"
#include "MotorController.hpp"
#include "NavigatorController.hpp"
#include "RobotOdometry.hpp"
#include "SlamController.hpp"
#include "UserInputController.hpp"

using namespace std;
using namespace Manhattan;

void AddKinematics(const shared_ptr<Core::App>& app)
{
    const auto motor = app->AddController<Core::MotorController>();
    const auto odometry = app->AddController<Core::RobotOdometry>();
}

int main(const int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    const auto app = make_shared<Core::App>();

    AddKinematics(app);

    app->AddController<Core::ImuComponent>()->Enable();

    app->AddController<Core::LineController>();
    app->AddController<Core::LidarController>();

    app->AddController<Core::UserInputController>();
    app->AddController<Core::SlamController>();
    app->AddController<Core::NavigatorController>();
    app->AddController<Core::FollowerController>();
    app->AddController<Core::ExplorerController>();

    app->GetController<Core::FollowerController>()->Enable();
    // app->GetController<Core::ExplorerController>()->Enable();

    app->Run();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
