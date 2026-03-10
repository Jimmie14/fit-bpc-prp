#include <rclcpp/rclcpp.hpp>
#include "MotorController.hpp"
#include "Kinematics.hpp"
#include "App.hpp"

using namespace std;

shared_ptr<rclcpp::executors::MultiThreadedExecutor> init_nodes()
{
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // nodes
    auto motor_node = std::make_shared<rclcpp::Node>("motor");

    auto motor_controller = std::make_shared<MotorController>(motor_node);
    motor_controller->set_speed(1, 0);

    executor->add_node(motor_node);

    return executor;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = init_nodes();

    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
