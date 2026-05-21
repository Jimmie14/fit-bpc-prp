#pragma once

#include "Common/RosDeviceDriver.hpp"
#include "Messages/RobotMode.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace Manhattan::core {

using namespace Manhattan::messages;

class ButtonsDriver final : public RosDeviceDriver {
public:
    explicit ButtonsDriver(const App& app);

protected:
    void OnEnable() override;

    void OnDisable() override;

private:
    Subscription<std_msgs::msg::UInt8>::SharedPtr _subscriber;
    RobotMode _mode = { .reverse = false };

    void OnButtons(const std_msgs::msg::UInt8& msg);
};
} // namespace Manhattan::Core
