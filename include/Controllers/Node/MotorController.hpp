#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "BaseController.h"

namespace Manhattan::Core
{
    class MotorController : public BaseController
    {
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _publisher;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr _subscriber;

        rclcpp::TimerBase::SharedPtr _timer;
        std_msgs::msg::UInt8MultiArray _msg;

        void Update() const;

        void SubscriberCallback(std_msgs::msg::UInt32MultiArray::SharedPtr msg) const;

    public:
        explicit MotorController(const App& app);

        void SetForce(double leftAngular, double rightAngular);
    };
}
