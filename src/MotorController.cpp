#include <rclcpp/rclcpp.hpp>

class MotorController {
public:
    // Constructor takes a shared_ptr to an existing node instead of creating one.
    MotorController(const rclcpp::Node::SharedPtr &node, double freq)
        : node_(node), start_time_(node_->now()) 
    {
        auto publisher = "/bpc_prp_robot/set_motor_speeds";
        auto subscriber = "/bpc_prp_robot/encoders";

        // Initialize the publisher
        publisher_ = node_->create_publisher<std_msgs::msg::UInt8>(publisher, 1);

        // Initialize the subscriber
        subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
            subscriber, 1, std::bind(&MotorController::subscriber_callback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "Node setup complete for publisher: %s and subscriber: %s", publisher.c_str(), subscriber.c_str());
    }

private:
    void subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "Received: %u", msg->data);
    }

    void publish_message(float value_to_publish) {
        auto msg = std_msgs::msg::UInt8();
        msg.data = static_cast<uint8_t>(value_to_publish);
        publisher_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published: %u", msg.data);
    }

    // Shared pointer to the main ROS node
    rclcpp::Node::SharedPtr node_;

    // Publisher, subscriber, and timer
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
};