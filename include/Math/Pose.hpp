#pragma once

#include "Vector2.hpp"

#include <geometry_msgs/msg/pose.hpp>

namespace Manhattan::Core {
struct Pose {
    Vector2 position;
    Vector2 forward;
    double rotation;

    Pose()
        : position(Vector2::Zero())
        , forward(Vector2(std::cos(M_PI * 0.5), std::sin(M_PI * 0.5)))
        , rotation(0.0)
    {
    }

    explicit Pose(const Vector2 position, const double rotation)
        : position(position)
        , rotation(rotation)
    {
        forward = Vector2(std::cos(rotation + M_PI * 0.5), std::sin(rotation + M_PI * 0.5));
    }

    Pose operator-(const Pose& other) const
    {
        return Pose(position - other.position, rotation - other.rotation);
    }

    Pose operator+(const Pose& other) const
    {
        return Pose(position + other.position, rotation + other.rotation);
    }

    static Pose Zero()
    {
        return Pose(Vector2::Zero(), 0.0);
    }

    static Pose Identity()
    {
        return Pose(Vector2::Zero(), 0.0);
    }

    void TransformPointsInplace(std::vector<Vector2>& points) const
    {
        const auto c = std::cos(rotation);
        const auto s = std::sin(rotation);

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
        const auto c = std::cos(rotation);
        const auto s = std::sin(rotation);

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
        const double halfTheta = (rotation + M_PI * 0.5) * 0.5;

        const double qw = std::cos(halfTheta);
        const double qz = std::sin(halfTheta);

        auto msg = geometry_msgs::msg::Pose();

        msg.position.x = position.x;
        msg.position.y = position.y;
        msg.position.z = 0.0;

        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = qz;
        msg.orientation.w = qw;

        return msg;
    }

    [[nodiscard]] std::string ToString() const
    {
        return "(pos=" + position.ToString() + ", theta=" + std::to_string(rotation) + ")";
    }
};

}