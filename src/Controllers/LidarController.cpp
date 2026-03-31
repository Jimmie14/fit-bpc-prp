#include "LidarController.hpp"

#include "Point.hpp"

namespace Manhattan::Core
{
    constexpr auto LIDAR_TOPIC = "/bpc_prp_robot/lidar";

    LidarController::LidarController(const App& app) : BaseController(app), _lidar_filter(.2, 50)
    {
        _lidar_subscriber = _node->create_subscription<sensor_msgs::msg::LaserScan>(
            LIDAR_TOPIC, 1, [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
            { LidarCallback(msg); }
        );

        _line_publisher = _node->create_publisher<visualization_msgs::msg::Marker>("~/visualization_marker", 10);
    }

    void LidarController::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
    {
        std::vector<Point> points;
        points.reserve(msg->ranges.size());

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max) continue;

            points.push_back(Point::FromAngle(msg->angle_min + i * msg->angle_increment) * range);
        }

        auto lines = _lidar_filter.Filter(points);

        std::cout << lines.size() << std::endl;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "lidar";
        marker.header.stamp = _node->now();
        marker.ns = "lines";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05; // line width in meters
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (const auto& line : lines) {
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = line.start.x;
            p_start.y = line.start.y;
            p_start.z = 0.0;
            p_end.x = line.end.x;
            p_end.y = line.end.y;
            p_end.z = 0.0;
            marker.points.push_back(p_start);
            marker.points.push_back(p_end);
        }

        _line_publisher->publish(marker);
    }
}
