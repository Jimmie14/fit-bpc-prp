#include <rclcpp/rclcpp.hpp>
#include "MotorController.hpp"
#include "Kinematics.hpp"
#include "App.hpp"

using namespace std;

void AddMotor(const shared_ptr<Manhattan::Core::App>& app)
{
    auto motor = app->AddController<MotorController>();
    motor->SetSpeed(.4, .4);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto app = make_shared<Manhattan::Core::App>();

    AddMotor(app);

    app->Run();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
