#pragma once

#include "RobotMode.hpp"
#include "RosDeviceDriver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace Manhattan::Core {
class ButtonsDriver final : public RosDeviceDriver {
public:
    explicit ButtonsDriver(const App& app);

    void OnEnable() override;

    void OnDisable() override;

private:
    Subscription<std_msgs::msg::UInt8>::SharedPtr _subscriber;
    RobotMode _mode = { .reverse = false };

    void OnButtons(const std_msgs::msg::UInt8& msg);
};
} // namespace Manhattan::Core
