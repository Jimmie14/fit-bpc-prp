#pragma once

#include "RosDeviceDriver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace Manhattan::Core {
class ImuDriver final : public RosDeviceDriver {
public:
    explicit ImuDriver(const App& app);

    void OnEnable() override;

    void OnDisable() override;

private:
    Subscription<sensor_msgs::msg::Imu>::SharedPtr _imuSubscriber;
    Publisher<sensor_msgs::msg::Imu>::SharedPtr _imuPublisher;

    void OnImu(const sensor_msgs::msg::Imu::SharedPtr& msg) const;
};
} // namespace Manhattan::Core
