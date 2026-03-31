#pragma once
#include "Line.hpp"
#include "Point.hpp"

namespace Manhattan::Core
{
    class LidarFilter
    {
        double _maxDistanceBetweenPoints;
        double _angleThreshold;
    public:
        explicit LidarFilter(double minDistance, double angleThreshold)
        : _maxDistanceBetweenPoints(minDistance), _angleThreshold(angleThreshold * (M_PI / 180.0)) {}

        [[nodiscard]] std::vector<Line> Filter(const std::vector<Point>& points) const
        {
            std::vector<Line> lines;
            if (points.empty()) return lines;

            std::vector<Point> currentGroup;
            currentGroup.push_back(points[0]);

            for (size_t i = 1; i < points.size(); ++i) {
                double dist = Point::Distance(points[i-1], points[i]);
                if (dist <= _maxDistanceBetweenPoints) {
                    currentGroup.push_back(points[i]);
                } else {
                    if (currentGroup.size() >= 2) {
                        auto newLines = Line::FitLine(currentGroup, _angleThreshold);
                        lines.insert(lines.end(), newLines.begin(), newLines.end());
                    }
                    currentGroup.clear();
                    currentGroup.push_back(points[i]);
                }
            }
            if (currentGroup.size() >= 2) {
                auto newLines = Line::FitLine(currentGroup, _angleThreshold);
                lines.insert(lines.end(), newLines.begin(), newLines.end());
            }
            return lines;
        }
    };
}
