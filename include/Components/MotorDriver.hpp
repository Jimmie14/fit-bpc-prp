#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "RosDeviceDriver.hpp"

namespace Manhattan::Core {
class MotorDriver final : public RosDeviceDriver {
    Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _publisher;

    TimerBase::SharedPtr _timer;
    std_msgs::msg::UInt8MultiArray _msg;
public:
    explicit MotorDriver(const App& app);

    void OnEnable() override;

    void OnDisable() override;

    void SetForce(double leftAngular, double rightAngular);
};
} // namespace Manhattan::Core
