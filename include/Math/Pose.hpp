#pragma once

#include "Vector2.hpp"

#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace Manhattan::math {

struct Twist {
    double linear;
    double angular;

    Twist() = default;

    explicit Twist(const double linear, const double angular)
        : linear(linear)
        , angular(angular)
    {
    }

    [[nodiscard]] static Twist fromRosTwistMessage(const geometry_msgs::msg::Twist& twist)
    {
        return Twist(twist.linear.x, twist.angular.z);
    }

    [[nodiscard]] static Twist zero()
    {
        return Twist(0.0, 0.0);
    }

    [[nodiscard]] std::string toString() const
    {
        return "(linear=" + std::to_string(linear) + ", angular=" + std::to_string(angular) + ")";
    }
};

inline std::ostream& operator<<(std::ostream& os, const Twist& twist)
{
    os << twist.toString();
    return os;
}

struct Pose {
    Vector2 position;
    double theta;

    Pose()
        : position(Vector2::zero())
        , theta(0.0)
    {

    }

    explicit Pose(const Vector2 position, const double theta)
        : position(position)
        , theta(theta)
    {

    }

    Pose operator-(const Pose& other) const
    {
        return Pose(position - other.position, theta - other.theta);
    }

    Pose operator+(const Pose& other) const
    {
        return Pose(position + other.position, theta + other.theta);
    }

    Pose operator*(const double scalar) const
    {
        return Pose(position * scalar, theta * scalar);
    }

    static Pose zero()
    {
        return Pose(Vector2::zero(), 0.0);
    }

    [[nodiscard]] Vector2 forward() const
    {
        return Vector2(std::cos(theta), std::sin(theta));
    }

    [[nodiscard]] Pose Normalized() const
    {
        auto angle = fmod(theta, 2.0 * M_PI);
        if (angle < 0) angle += 2.0 * M_PI;

        return Pose(position, angle);
    }

    void TransformPointsInplace(std::vector<Vector2>& points) const
    {
        const auto c = std::cos(theta);
        const auto s = std::sin(theta);

        std::vector<Vector2> result;
        result.reserve(points.size());

        for (auto& p : points) {
            const auto x = p.x;
            const auto y = p.y;

            p.x = position.x + x * c - y * s;
            p.y = position.y + x * s + y * c;
        }
    }

    void InverseTransformPointsInplace(std::vector<Vector2>& points) const
    {
        const auto c = std::cos(theta);
        const auto s = std::sin(theta);

        std::vector<Vector2> result;
        result.reserve(points.size());

        for (auto& p : points) {
            const auto dx = p.x - position.x;
            const auto dy = p.y - position.y;

            p.x = dx * c + dy * s;
            p.y = -dx * s + dy * c;
        }
    }

    Vector2 transformInverse(const Vector2& point) const
    {
        const auto c = std::cos(theta);
        const auto s = std::sin(theta);

        const auto dx = point.x - position.x;
        const auto dy = point.y - position.y;

        Vector2 result;

        result.x = dx * c + dy * s;
        result.y = -dx * s + dy * c;

        return result;
    }

    [[nodiscard]] geometry_msgs::msg::Pose ToRosPoseMessage() const
    {
        const double halfTheta = theta * 0.5;

        auto msg = geometry_msgs::msg::Pose();

        msg.position.x = position.x;
        msg.position.y = position.y;
        msg.position.z = 0.0;

        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = std::sin(halfTheta);
        msg.orientation.w = std::cos(halfTheta);

        return msg;
    }

    [[nodiscard]] static Pose fromRosPoseMessage(const geometry_msgs::msg::Pose& pose)
    {
        Pose result;

        result.position.x = pose.position.x;
        result.position.y = pose.position.y;
        result.theta = 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);

        return result;
    }

    [[nodiscard]] std::string ToString() const
    {
        return "(pos=" + position.toString() + ", theta=" + std::to_string(theta) + ")";
    }
};

}