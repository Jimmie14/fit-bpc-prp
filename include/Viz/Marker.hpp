#pragma once

#include "Nav.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace Manhattan::viz::marker {

using namespace viz::nav;

inline visualization_msgs::msg::Marker clear(const std::string& frame_id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.stamp = Clock().now();
    marker.header.frame_id = frame_id;
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;

    return marker;
}


inline visualization_msgs::msg::Marker direction(
    const tf2::Vector3& start,
    const tf2::Vector3& direction,
    const std::string& frame_id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = Clock().now();

    marker.ns = "direction";

    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.points.push_back(toPoint(start));
    marker.points.push_back(toPoint(start + direction));

    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    return marker;
}

inline visualization_msgs::msg::Marker point(
    const tf2::Vector3& position,
    const std::string& frame_id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.stamp = Clock().now();
    marker.header.frame_id = frame_id;
    marker.ns = "point";

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = toPose(position, 0.0f);

    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    return marker;
}

inline visualization_msgs::msg::Marker text(
    const tf2::Vector3& position,
    const std::string& text,
    const std::string& frame_id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.stamp = Clock().now();
    marker.header.frame_id = frame_id;
    marker.ns = "text";
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = toPose(position, 0.0f);
    marker.pose.position.z = 0.18;
    marker.scale.z = 0.14;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.text = text;

    return marker;
}

} // namespace viz
