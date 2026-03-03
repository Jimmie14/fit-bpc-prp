#include <rclcpp/rclcpp.hpp>
#include "example.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto node1 = std::make_shared<rclcpp::Node>("motor");

    // Create instances of RosExampleClass using the existing nodes
    auto motor_controller = std::make_shared<MotorController>(node1);

    // Add nodes to the executor
    executor->add_node(node1);

    // Run the executor (handles callbacks for nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
