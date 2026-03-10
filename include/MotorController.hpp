
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

using namespace std;

class MotorController {
    // Shared pointer to the main ROS node
    rclcpp::Node::SharedPtr node_;

    // Publisher, subscriber
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;

    // Start time for uptime calculation
    rclcpp::Time start_time_;

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

    void set_speed(double left, double right) {
        left = clamp(left, -1.0, 1.0);
        right = clamp(right, -1.0, 1.0);

        auto leftInt = static_cast<uint8_t>((left * .5 + 1) * 255);
        auto rightInt = static_cast<uint8_t>((right * .5 + 1) * 255);

        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.push_back(leftInt);
        msg.data.push_back(rightInt);

        publisher_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published: left => %u, right => %u", msg.data[0], msg.data[1]);
    }

private:
    void subscriber_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        auto length = msg->data.size();

        RCLCPP_INFO(node_->get_logger(), "Length: %lu", length);

        if (length >= 1)
            RCLCPP_INFO(node_->get_logger(), "Left wheel turned: %u", msg->data[0]);

        if (length >= 2)
            RCLCPP_INFO(node_->get_logger(), "Right wheel turned: %u", msg->data[1]);
    }
};
