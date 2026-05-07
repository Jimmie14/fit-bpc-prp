#pragma once

#include "Vector2.hpp"

#include <geometry_msgs/msg/pose.hpp>

namespace Manhattan::Core {

using namespace Manhattan::Math;

struct Pose {
    Vector2 position;
    Vector2 forward;
    double theta;

    Pose()
        : position(Vector2::Zero())
        , forward(Vector2(std::cos(0.0), std::sin(0.0)))
        , theta(0.0)
    {
    }

    explicit Pose(const Vector2 position, const double theta)
        : position(position)
        , theta(theta)
    {
        forward = Vector2(std::cos(theta), std::sin(theta));
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

    static Pose Zero()
    {
        return Pose(Vector2::Zero(), 0.0);
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

    [[nodiscard]] geometry_msgs::msg::Pose ToRosPoseMessage() const
    {
        const double yaw = theta + M_PI * 0.5;
        const double halfYaw = yaw * 0.5;

        auto msg = geometry_msgs::msg::Pose();

        msg.position.x = position.x;
        msg.position.y = position.y;
        msg.position.z = 0.0;

        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = std::sin(halfYaw);
        msg.orientation.w = std::cos(halfYaw);

        return msg;
    }

    [[nodiscard]] static Pose FromRosPoseMessage(const geometry_msgs::msg::Pose& pose)
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