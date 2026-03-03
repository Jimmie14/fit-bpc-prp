
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

class MotorController {
public:
    // Constructor takes a shared_ptr to an existing node instead of creating one.
    MotorController(const rclcpp::Node::SharedPtr &node)
        : node_(node), start_time_(node_->now())
    {
        auto publisher = "/bpc_prp_robot/set_motor_speeds";
        auto subscriber = "/bpc_prp_robot/encoders";

        // Initialize the publisher
        publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(publisher, 1);

        // Initialize the subscriber
        subscriber_ = node_->create_subscription<std_msgs::msg::UInt32MultiArray>(
            subscriber, 1, std::bind(&MotorController::subscriber_callback, this, std::placeholders::_1));
    }

private:
    void subscriber_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "Received: %u", msg->data[0]);
    }

    void publish_message(float value_to_publish) {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.push_back(static_cast<uint8_t>(value_to_publish));
        publisher_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published: %u", msg.data[0]);
    }

    // Shared pointer to the main ROS node
    rclcpp::Node::SharedPtr node_;

    // Publisher, subscriber
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;

    // Start time for uptime calculation
    rclcpp::Time start_time_;
};
