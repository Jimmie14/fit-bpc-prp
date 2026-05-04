#pragma once

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

    void OnButtons(const std_msgs::msg::UInt8& msg) const;
};
} // namespace Manhattan::Core
