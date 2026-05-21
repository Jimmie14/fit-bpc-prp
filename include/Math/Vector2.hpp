#pragma once

#include <cmath>
#include <string>
#include <tf2/LinearMath/Vector3.hpp>
#include <tuple>
#include <vector>

namespace Manhattan::math {

struct Vector2;

struct Vector2i {
    int x, y;

    Vector2i() = default;

    Vector2i(const int x, const int y)
        : x(x)
        , y(y)
    {

    }

    Vector2i(const std::pair<int, int>& point)
        : x(point.first)
        , y(point.second)
    {

    }

    Vector2i operator+(const Vector2i& other) const
    {
        return Vector2i(x + other.x, y + other.y);
    }

    Vector2i operator-(const Vector2i& other) const
    {
        return Vector2i(x - other.x, y - other.y);
    }

    bool operator<(const Vector2i& other) const
    {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

    bool operator==(const Vector2i& other) const
    {
        return std::tie(x, y) == std::tie(other.x, other.y);
    }

    bool operator!=(const Vector2i& other) const
    {
        return !(*this == other);
    }

    [[nodiscard]] std::string toString() const
    {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }

    static Vector2i zero()
    {
        return Vector2i(0, 0);
    }

    static Vector2i up()
    {
        return Vector2i(0, 1);
    }

    static Vector2i down()
    {
        return Vector2i(0, -1);
    }

    static Vector2i left()
    {
        return Vector2i(-1, 0);
    }

    static Vector2i right()
    {
        return Vector2i(1, 0);
    }

    static std::vector<Vector2i> directions()
    {
        return { up(), right(), down(), left() };
    }

    static std::vector<Vector2i> EightDirections()
    {
        return { up(), right(), down(), left(), up() + left(), up() + right(), down() + right(), down() + left() };
    }

    static Vector2i round(const Vector2& vector);
};

struct Vector2iHash {
    size_t operator()(const Vector2i& vector) const noexcept
    {
        const size_t h1 = std::hash<int> {}(vector.x);
        const size_t h2 = std::hash<int> {}(vector.y);

        return h1 ^ (h2 << 1);
    }
};

struct Vector2 {
    double x = 0;
    double y = 0;

    explicit Vector2()
    {
    }
    explicit Vector2(double x, double y)
        : x(x)
        , y(y)
    {
    }
    explicit Vector2(const Vector2i& v)
        : x(v.x)
        , y(v.y)
    {
    }

    explicit Vector2(const tf2::Vector3& v)
        : x(v.x())
        , y(v.y())
    {
    }

    Vector2 operator+(const Vector2& other) const
    {
        return Vector2(x + other.x, y + other.y);
    }

    Vector2 operator-(const Vector2& other) const
    {
        return Vector2(x - other.x, y - other.y);
    }

    Vector2 operator*(const double scalar) const
    {
        return Vector2(x * scalar, y * scalar);
    }

    Vector2 operator/(const double scalar) const
    {
        return Vector2(x / scalar, y / scalar);
    }

    Vector2 operator-() const
    {
        return Vector2(-x, -y);
    }

    static Vector2 Rotate(const Vector2& p, const double angle)
    {
        const double cosA = std::cos(angle);
        const double sinA = std::sin(angle);
        return Vector2(p.x * cosA - p.y * sinA, p.x * sinA + p.y * cosA);
    }

    static Vector2 FromAngle(const double angle)
    {
        return Vector2(std::cos(angle), std::sin(angle));
    }

    static double distance(const Vector2& p1, const Vector2& p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    static Vector2 clamp(const Vector2& value, const Vector2& min, const Vector2& max)
    {
        return Vector2(
            std::clamp(value.x, min.x, max.x),
            std::clamp(value.y, min.y, max.y)
        );
    }

    static Vector2 lerp(const Vector2& a, const Vector2& b, double t)
    {
        t = std::max(0.0, std::min(1.0, t));
        return a + (b - a) * t;
    }

    static double dot(const Vector2& p1, const Vector2& p2)
    {
        return p1.x * p2.x + p1.y * p2.y;
    }

    static double Cross(const Vector2& p1, const Vector2& p2)
    {
        return p1.x * p2.y - p1.y * p2.x;
    }

    static Vector2 Perpendicular(const Vector2& p)
    {
        return Vector2(p.y, -p.x);
    }

    static double signedAngle(const Vector2& from, const Vector2& to)
    {
        const double dot = Vector2::dot(from, to);
        const double det = from.x * to.y - from.y * to.x;

        return std::atan2(det, dot);
    }

    [[nodiscard]] double sqrMagnitude() const
    {
        return x * x + y * y;
    }

    [[nodiscard]] double magnitude() const
    {
        return std::sqrt(sqrMagnitude());
    }

    [[nodiscard]] Vector2 normalized() const
    {
        const auto c = magnitude();

        return Vector2(x / c, y / c);
    }

    [[nodiscard]] tf2::Vector3 toTf2() const
    {
        return { x, y, 0.0f };
    }

    static Vector2 zero()
    {
        return Vector2(0, 0);
    }

    static Vector2 one()
    {
        return Vector2(1, 1);
    }

    [[nodiscard]] std::string toString() const
    {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }
};

struct RayHit {
    Vector2 hit;
    Vector2 normal;
};

} // namespace Manhattan::Core
