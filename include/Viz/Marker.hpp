#pragma once

#include "Nav.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "Math/Pose.hpp"

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

inline visualization_msgs::msg::Marker twist(const math::Pose& pose, const math::Twist& twist, const std::string& frame_id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = Clock().now();

    marker.ns = "twist";

    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    // LINE_LIST width
    marker.scale.x = 0.03;

    // White
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    constexpr double linear_scale  = 0.5;
    constexpr double angular_scale = 0.4;

    auto make_point = [](double x, double y)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.0;
        return p;
    };

    const double px = pose.position.x;
    const double py = pose.position.y;

    // ------------------------------------------------------------------------
    // Linear velocity arrow
    // ------------------------------------------------------------------------

    const double linear_mag = std::abs(twist.linear);

    if (linear_mag > 1e-3)
    {
        const double dir = (twist.linear >= 0.0) ? 1.0 : -1.0;

        const double len = linear_mag * linear_scale;

        const double fx = pose.forward().x;
        const double fy = pose.forward().y;

        const double ex = px + fx * len * dir;
        const double ey = py + fy * len * dir;

        // Main shaft
        marker.points.push_back(make_point(px, py));
        marker.points.push_back(make_point(ex, ey));

        // Arrow head only if strong enough
        if (linear_mag > 0.15)
        {
            const double head_len   = std::min(0.25, len * 0.35);
            const double head_width = head_len * 0.6;

            // perpendicular
            const double nx = -fy;
            const double ny =  fx;

            const double bx = ex - fx * head_len * dir;
            const double by = ey - fy * head_len * dir;

            // left wing
            marker.points.push_back(make_point(ex, ey));
            marker.points.push_back(
                make_point(
                    bx + nx * head_width,
                    by + ny * head_width));

            // right wing
            marker.points.push_back(make_point(ex, ey));
            marker.points.push_back(
                make_point(
                    bx - nx * head_width,
                    by - ny * head_width));
        }
    }

    // ------------------------------------------------------------------------
    // Angular velocity arc
    // ------------------------------------------------------------------------

    const double angular_mag = std::abs(twist.angular);

    if (angular_mag > 1e-3)
    {
        const double sign = (twist.angular >= 0.0) ? 1.0 : -1.0;

        // Radius grows with angular velocity
        const double radius = 0.25 + angular_mag * angular_scale;

        // Arc angle also grows slightly
        const double arc_angle =
            std::min(M_PI * 1.7, angular_mag * 1.2 + M_PI * 0.4);

        constexpr int segments = 24;

        for (int i = 0; i < segments; ++i)
        {
            const double t0 =
                pose.theta + sign * (-arc_angle * 0.5 +
                arc_angle * (double(i) / segments));

            const double t1 =
                pose.theta + sign * (-arc_angle * 0.5 +
                arc_angle * (double(i + 1) / segments));

            const double x0 = px + std::cos(t0) * radius;
            const double y0 = py + std::sin(t0) * radius;

            const double x1 = px + std::cos(t1) * radius;
            const double y1 = py + std::sin(t1) * radius;

            marker.points.push_back(make_point(x0, y0));
            marker.points.push_back(make_point(x1, y1));
        }

        // Arc arrow head
        const double end_theta =
            pose.theta + sign * (arc_angle * 0.5);

        const double ex = px + std::cos(end_theta) * radius;
        const double ey = py + std::sin(end_theta) * radius;

        // tangent direction
        const double tx = -std::sin(end_theta) * sign;
        const double ty =  std::cos(end_theta) * sign;

        const double head_len = 0.12;

        const double left_x =
            ex - tx * head_len + (-ty) * head_len * 0.6;
        const double left_y =
            ey - ty * head_len + ( tx) * head_len * 0.6;

        const double right_x =
            ex - tx * head_len - (-ty) * head_len * 0.6;
        const double right_y =
            ey - ty * head_len - ( tx) * head_len * 0.6;

        marker.points.push_back(make_point(ex, ey));
        marker.points.push_back(make_point(left_x, left_y));

        marker.points.push_back(make_point(ex, ey));
        marker.points.push_back(make_point(right_x, right_y));
    }

    return marker;
}

inline visualization_msgs::msg::Marker path(const vector<math::Vector2>& points, const std::string& frame_id, const double scale = 0.02)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = Clock().now();

    marker.ns = "path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = scale;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    marker.points.reserve(points.size() * 2);

    for (size_t i = 0; i + 1 < points.size(); ++i)
    {
        marker.points.push_back(toPoint(points[i].toTf2()));
        marker.points.push_back(toPoint(points[i + 1].toTf2()));
    }

    return marker;
}

inline visualization_msgs::msg::Marker line(const math::Vector2& p1, const math::Vector2& p2, const std::string& frame_id, const double scale = 0.02)
{
    return path({ p1, p2 }, frame_id, scale);
}

inline std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0f)
{
    std_msgs::msg::ColorRGBA color;

    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}


class MarkerArrayBuilder {
public:
    visualization_msgs::msg::MarkerArray array;
    std_msgs::msg::ColorRGBA color;


    int GetNextMarkerId()
    {
        return _nextId++;
    }

    void add(visualization_msgs::msg::Marker marker)
    {
        marker.color = color;
        marker.id = _nextId++;
        array.markers.push_back(marker);
    }


private:
    int _nextId = 0;

};

} // namespace viz
