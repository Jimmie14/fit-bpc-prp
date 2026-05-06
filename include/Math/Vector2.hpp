#pragma once

#include <cmath>
#include <string>
#include <tf2/LinearMath/Vector3.hpp>
#include <tuple>
#include <vector>

namespace Manhattan::Math {
struct Vector2Int {
    int x, y;

    Vector2Int() = default;

    Vector2Int(const int x, const int y)
        : x(x)
        , y(y)
    {

    }

    Vector2Int(const std::pair<int, int>& point)
        : x(point.first)
        , y(point.second)
    {

    }

    Vector2Int operator+(const Vector2Int& other) const
    {
        return Vector2Int(x + other.x, y + other.y);
    }

    Vector2Int operator-(const Vector2Int& other) const
    {
        return Vector2Int(x - other.x, y - other.y);
    }

    bool operator<(const Vector2Int& other) const
    {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

    bool operator==(const Vector2Int& other) const
    {
        return std::tie(x, y) == std::tie(other.x, other.y);
    }

    bool operator!=(const Vector2Int& other) const
    {
        return !(*this == other);
    }

    [[nodiscard]] std::string toString() const
    {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }

    static Vector2Int zero()
    {
        return Vector2Int(0, 0);
    }

    static Vector2Int up()
    {
        return Vector2Int(0, 1);
    }

    static Vector2Int down()
    {
        return Vector2Int(0, -1);
    }

    static Vector2Int left()
    {
        return Vector2Int(-1, 0);
    }

    static Vector2Int right()
    {
        return Vector2Int(1, 0);
    }

    static std::vector<Vector2Int> directions()
    {
        return { up(), right(), down(), left() };
    }

    static std::vector<Vector2Int> EightDirections()
    {
        return { up(), right(), down(), left(), up() + left(), up() + right(), down() + right(), down() + left() };
    }
};

struct Vector2IntHash {
    size_t operator()(const Vector2Int& vector) const noexcept
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
    explicit Vector2(const Vector2Int& v)
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

    static double Distance(const Vector2& p1, const Vector2& p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    static Vector2 Lerp(const Vector2& a, const Vector2& b, double t)
    {
        t = std::max(0.0, std::min(1.0, t));
        return a + (b - a) * t;
    }

    static double Dot(const Vector2& p1, const Vector2& p2)
    {
        return p1.x * p2.x + p1.y * p2.y;
    }

    static double SignedAngle(const Vector2& from, const Vector2& to)
    {
        const double dot = Dot(from, to);
        const double det = from.x * to.y - from.y * to.x;

        return std::atan2(det, dot);
    }

    [[nodiscard]] double SqrMagnitude() const
    {
        return x * x + y * y;
    }

    [[nodiscard]] double Magnitude() const
    {
        return std::sqrt(SqrMagnitude());
    }

    [[nodiscard]] Vector2 Normalized() const
    {
        const auto c = Magnitude();

        return Vector2(x / c, y / c);
    }

    [[nodiscard]] tf2::Vector3 ToTf2() const
    {
        return { x, y, 0.0f };
    }

    static Vector2 Zero()
    {
        return Vector2(0, 0);
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
