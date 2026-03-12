#include <rclcpp/rclcpp.hpp>

#include "App.h"
#include "../include/Controllers/Node/MotorController.h"

using namespace std;
using namespace Manhattan;

void AddMotor(const shared_ptr<Core::App>& app)
{
    const auto motor = app->AddController<Core::MotorController>();
    motor->SetForce(.4, .4);
}

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    const auto app = make_shared<Core::App>();

    AddMotor(app);

    app->Run();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
