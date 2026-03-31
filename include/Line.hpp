#pragma once
#include "Point.hpp"

namespace Manhattan::Core
{
    struct Line
    {
        Point start;
        Point end;

        explicit Line(const Point start, const Point end) : start(start), end(end) {}

        double Length() const {
            return Point::Distance(start, end);
        }

        static double AngleBetween(const Point& a, const Point& b, const Point& c) {
            const double dx1 = b.x - a.x;
            const double dy1 = b.y - a.y;
            const double dx2 = c.x - b.x;
            const double dy2 = c.y - b.y;
            const double dot = dx1 * dx2 + dy1 * dy2;
            const double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            const double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (mag1 == 0 || mag2 == 0) return 0.0;
            double cosTheta = dot / (mag1 * mag2);

            // Clamp to [-1, 1] to avoid NaN due to floating point errors
            cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
            return std::acos(cosTheta);
        }

        static std::vector<Line> FitLine(const std::vector<Point>& points, const double angleThreshold) {
            std::vector<Line> lines;
            if (points.size() < 2) return lines;

            size_t startIdx = 0;
            for (size_t i = 1; i < points.size() - 1; ++i) {
                double angle = AngleBetween(points[i-1], points[i], points[i+1]);
                if (angle > angleThreshold) {
                    lines.emplace_back(points[startIdx], points[i]);
                    startIdx = i;
                }
            }

            lines.emplace_back(points[startIdx], points.back());
            return lines;
        }
    };
}
