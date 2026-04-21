#pragma once

#include "BaseController.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace Manhattan::Core {
class ImuComponent final : public BaseController {
public:
    explicit ImuComponent(const App& app);

    void OnEnable() override;

    void OnDisable() override;
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imuSubscriber;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imuPublisher;

    void OnImu(const sensor_msgs::msg::Imu::SharedPtr& msg) const;
};
} // namespace Manhattan::Core
