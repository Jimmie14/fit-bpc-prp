#pragma once
#include <cmath>

namespace Manhattan::Core
{
    struct Vector2Int
    {
        int x, y;

        Vector2Int operator+(const Vector2Int& other) const {
            return Vector2Int(x + other.x, y + other.y);
        }

        Vector2Int operator-(const Vector2Int& other) const {
            return Vector2Int(x - other.x, y - other.y);
        }

        bool operator<(const Vector2Int& other) const {
            return std::tie(x, y) < std::tie(other.x, other.y);
        }
    };

    struct Point
    {
        double x = 0;
        double y = 0;

        explicit Point() {}
        explicit Point(double x, double y) : x(x), y(y) {}

        Point operator+(const Point& other) const {
            return Point(x + other.x, y + other.y);
        }

        Point operator-(const Point& other) const {
            return Point(x - other.x, y - other.y);
        }

        Point operator*(const double scalar) const {
            return Point(x * scalar, y * scalar);
        }

        static Point Rotate(const Point& p, const double angle) {
            const double cosA = std::cos(angle);
            const double sinA = std::sin(angle);
            return Point(
                p.x * cosA - p.y * sinA,
                p.x * sinA + p.y * cosA
            );
        }

        static Point FromAngle(const double angle) {
            return Point(std::cos(angle), std::sin(angle));
        }

        static double Distance(const Point& p1, const Point& p2) {
            return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
        }

        double getX() const { return x; }
        double getY() const { return y; }
    };
}
