#include <rclcpp/rclcpp.hpp>

#include "App.h"
#include "Kinematics.hpp"
#include "../include/Controllers/Node/MotorController.h"

using namespace std;
using namespace Manhattan;

constexpr float WHEEL_BASE = 0.12;
constexpr float WHEEL_RADIUS = 0.033;
constexpr int32_t PULSES_PER_ROTATION = 550;

void AddKinematics(const shared_ptr<Core::App>& app)
{
    const auto motor = app->AddController<Core::MotorController>();
    auto kinematics = Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);

    app->SetMoveCallback([kinematics, motor](double linear, double angular) {
        auto robotSpeed = RobotSpeed(linear, angular);
        auto wheelSpeed = kinematics.inverse(robotSpeed);

        motor->SetForce(wheelSpeed.left / 30.0, wheelSpeed.right / 30.0);
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
