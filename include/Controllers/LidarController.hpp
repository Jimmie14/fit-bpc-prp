#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "BaseController.h"
#include "LidarFilter.hpp"

namespace Manhattan::Core
{
    class LidarController final : public BaseController
    {
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;

        LidarFilter _lidar_filter;
        std::vector<std::vector<Point>> _points;

        double _rotation = 0;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _line_publisher;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_array_publisher;

        void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr& msg);

    public:
        explicit LidarController(const App& app);
    };
}
