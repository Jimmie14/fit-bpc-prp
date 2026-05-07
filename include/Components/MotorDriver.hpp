#pragma once

#include "RobotMode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "RosDeviceDriver.hpp"

namespace Manhattan::Core {

struct MotorCommand {
    double linear;
    double angular;
};


class MotorDriver final : public RosDeviceDriver {
public:
    explicit MotorDriver(const App& app);

    void SetForce(double leftAngular, double rightAngular);

protected:
    void OnEnable() override;

    void OnDisable() override;
private:
    Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _publisher;

    TimerBase::SharedPtr _timer;
    std_msgs::msg::UInt8MultiArray _msg;
    RobotMode _mode = { .reverse = false };
};
} // namespace Manhattan::Core
