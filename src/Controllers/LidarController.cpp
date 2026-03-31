#include "LidarController.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/color_rgba.hpp>

#include "Point.hpp"

namespace Manhattan::Core
{
    constexpr auto LIDAR_TOPIC = "/bpc_prp_robot/lidar";
    constexpr auto ODOMETRY_TOPIC = "/odometry/filtered";

    LidarController::LidarController(const App& app) : BaseController(app), _lidar_filter(.05, 20)
    {
        _lidar_subscriber = _node->create_subscription<sensor_msgs::msg::LaserScan>(
            LIDAR_TOPIC, 1, [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
            { LidarCallback(msg); }
        );

        _odometry_subscriber = _node->create_subscription<nav_msgs::msg::Odometry>(
            ODOMETRY_TOPIC, 1, [this](nav_msgs::msg::Odometry::SharedPtr msg)
            {
                tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                _rotation = yaw;
            }
        );

        _marker_array_publisher = _node->create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization_marker_array", 10);
        _line_publisher = _node->create_publisher<visualization_msgs::msg::Marker>("~/visualization_marker", 10);
    }

    void LidarController::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
    {
        std::vector<Point> points;
        points.reserve(msg->ranges.size());

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max) continue;

            points.push_back(Point::FromAngle(i * msg->angle_increment + _rotation) * range);
        }

        auto lines = _lidar_filter.Filter(points);

        std::cout << lines.size() << std::endl;

        std::vector<std_msgs::msg::ColorRGBA> color_list;
        {
            std_msgs::msg::ColorRGBA c;
            c.r = 1.0; c.g = 0.0; c.b = 0.0; c.a = 1.0; // Red
            color_list.push_back(c);
            c.r = 0.0; c.g = 1.0; c.b = 0.0; c.a = 1.0; // Green
            color_list.push_back(c);
            c.r = 0.0; c.g = 0.0; c.b = 1.0; c.a = 1.0; // Blue
            color_list.push_back(c);
            c.r = 1.0; c.g = 1.0; c.b = 0.0; c.a = 1.0; // Yellow
            color_list.push_back(c);
            c.r = 1.0; c.g = 0.0; c.b = 1.0; c.a = 1.0; // Magenta
            color_list.push_back(c);
            c.r = 0.0; c.g = 1.0; c.b = 1.0; c.a = 1.0; // Cyan
            color_list.push_back(c);
            c.r = 1.0; c.g = 0.5; c.b = 0.0; c.a = 1.0; // Orange
            color_list.push_back(c);
            c.r = 0.5; c.g = 0.0; c.b = 1.0; c.a = 1.0; // Purple
            color_list.push_back(c);
            c.r = 0.0; c.g = 0.5; c.b = 1.0; c.a = 1.0; // Sky blue
            color_list.push_back(c);
            c.r = 0.5; c.g = 1.0; c.b = 0.0; c.a = 1.0; // Lime
            color_list.push_back(c);
        }

        // auto groups = _lidar_filter.FilterPoints(points);
        // _points = groups;
        //
        // visualization_msgs::msg::MarkerArray marker_array;
        // auto delete_marker = visualization_msgs::msg::Marker();
        // delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        // marker_array.markers.push_back(delete_marker);
        // int group_id = 0;
        // for (const auto& group : groups) {
        //     visualization_msgs::msg::Marker marker;
        //     marker.header.frame_id = "map";
        //     marker.header.stamp = _node->now();
        //     marker.ns = "dots";
        //     marker.id = group_id;
        //     marker.type = visualization_msgs::msg::Marker::POINTS;
        //     marker.action = visualization_msgs::msg::Marker::ADD;
        //     marker.scale.x = 0.005;
        //     marker.scale.y = 0.005;
        //
        //     // Assign color from the list
        //     std_msgs::msg::ColorRGBA color = color_list[group_id % color_list.size()];
        //     marker.color = color;
        //
        //     for (const auto& point : group) {
        //         geometry_msgs::msg::Point p;
        //         p.x = point.x;
        //         p.y = point.y;
        //         p.z = 0.0;
        //         marker.points.push_back(p);
        //     }
        //     marker_array.markers.push_back(marker);
        //     group_id++;
        // }
        // _marker_array_publisher->publish(marker_array);

        auto delete_marker = visualization_msgs::msg::Marker();
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

        _line_publisher->publish(delete_marker);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = _node->now();
        marker.ns = "lines";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;
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
