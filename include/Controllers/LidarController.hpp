#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "BaseController.h"
#include "LidarFilter.hpp"

namespace Manhattan::Core
{
    class LidarController final : public BaseController
    {
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscriber;
        LidarFilter _lidar_filter;
        std::vector<Line> _lines;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _line_publisher;

        void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr& msg);

    public:
        explicit LidarController(const App& app);
    };
}
