#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "../Math/Vector2.hpp"
#include "RosDeviceDriver.hpp"

namespace Manhattan::Core {
class LidarDriver final : public RosDeviceDriver {
    std::vector<Vector2> _points {};

    Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscriber;

    void LidarFilter(const sensor_msgs::msg::LaserScan::SharedPtr& msg);

public:
    explicit LidarDriver(const App& app);
};

struct LidarScan {
    const std::vector<Vector2>& points;
};

} // namespace Manhattan::Core
