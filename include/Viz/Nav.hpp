#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace Manhattan::viz::nav {

inline geometry_msgs::msg::Point toPoint(const tf2::Vector3& v)
{
    geometry_msgs::msg::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}

inline geometry_msgs::msg::Pose toPose(const tf2::Vector3& position, const float theta)
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(theta * 0.5f);
    pose.orientation.w = cos(theta * 0.5f);

    return pose;
}


}