
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "Controller.hpp"

using namespace std;

class MotorController : public Manhattan::Core::BaseController {
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _publisher;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr _subscriber;

    rclcpp::TimerBase::SharedPtr _timer;
    std_msgs::msg::UInt8MultiArray _msg;

public:
    static constexpr const char* TypeName = "MotorController";

    explicit MotorController(const rclcpp::Node::SharedPtr &node) : BaseController(node)
    {
        const auto publisher = "/bpc_prp_robot/set_motor_speeds";
        const auto subscriber = "/bpc_prp_robot/encoders";

        _publisher = _node->create_publisher<std_msgs::msg::UInt8MultiArray>(publisher, 1);
        _subscriber = _node->create_subscription<std_msgs::msg::UInt32MultiArray>(
            subscriber, 1, std::bind(&MotorController::subscriber_callback, this, std::placeholders::_1));

        _msg = std_msgs::msg::UInt8MultiArray();
        _msg.data.push_back(127);
        _msg.data.push_back(127);

        _timer = _node->create_wall_timer(
            100ms,
            [this]() { loopCallback(); }
        );
    }

    void SetSpeed(double left, double right)
    {
        left = clamp(left, -1.0, 1.0);
        right = clamp(right, -1.0, 1.0);

        auto leftVal = static_cast<uint8_t>((left * .5 + .5) * 255);
        auto rightVal = static_cast<uint8_t>((right * .5 + .5) * 255);

        _msg.data[0] = leftVal;
        _msg.data[1] = rightVal;
    }

private:
    void loopCallback() {
        _publisher->publish(_msg);
        RCLCPP_INFO(_node->get_logger(), "Published: left => %u, right => %u", _msg.data[0], _msg.data[1]);
    }

    void subscriber_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) const
    {
        // auto length = msg->data.size();
        //
        // if (length >= 1)
        //     RCLCPP_INFO(_node->get_logger(), "Left wheel turned: %u", msg->data[0]);
        //
        // if (length >= 2)
        //     RCLCPP_INFO(_node->get_logger(), "Right wheel turned: %u", msg->data[1]);
    }
};
