#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "BaseController.h"
#include "Vector2.hpp"

namespace Manhattan::Core {
class LidarController final : public BaseController {
    std::vector<Vector2> _points {};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscriber;

    std::function<void(const std::vector<Vector2>&)> _scanCallback;

    void LidarFilter(const sensor_msgs::msg::LaserScan::SharedPtr& msg);

public:
    explicit LidarController(const App& app);

    void SetScanCallback(const std::function<void(const std::vector<Vector2>&)>& scanCallback)
    {
        _scanCallback = scanCallback;
    }
};
} // namespace Manhattan::Core
