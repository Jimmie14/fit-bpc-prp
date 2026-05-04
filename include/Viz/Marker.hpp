#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace Viz {
inline geometry_msgs::msg::Point ToPoint(const tf2::Vector3& v)
{
    geometry_msgs::msg::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}

inline visualization_msgs::msg::Marker ToDirection(
    const tf2::Vector3& start,
    const tf2::Vector3& direction,
    const std::string& frame_id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();

    marker.ns = "direction_arrow";

    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.points.push_back(ToPoint(start));
    marker.points.push_back(ToPoint(start + direction));

    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    return marker;
}
} // namespace viz
