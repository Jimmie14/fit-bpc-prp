#include <rclcpp/rclcpp.hpp>

#include "App.h"
#include "Kinematics.hpp"
#include "MotorController.hpp"
#include "RobotOdometry.hpp"

using namespace std;
using namespace Manhattan;

void AddKinematics(const shared_ptr<Core::App>& app)
{
    const auto motor = app->AddController<Core::MotorController>();
    const auto odometry = app->AddController<Core::RobotOdometry>();

    auto kinematics = odometry->GetKinematics();

    app->SetMoveCallback([kinematics, motor](double linear, double angular) {
        auto robotSpeed = RobotSpeed(linear, angular);
        auto wheelSpeed = kinematics.inverse(robotSpeed);

        motor->SetForce(wheelSpeed.left, wheelSpeed.right);
    });

    app->SetStopCallback([motor] {
        motor->SetForce(0, 0);
    });
}

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    const auto app = make_shared<Core::App>();

    AddKinematics(app);

    app->Run();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
