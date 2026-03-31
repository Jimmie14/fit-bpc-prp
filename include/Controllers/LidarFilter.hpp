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

            for (auto group :  FilterPoints(points))
            {
                auto newLines = Line::FitLine(group, _angleThreshold);
                lines.insert(lines.end(), newLines.begin(), newLines.end());
            }

            return lines;
        }

    private:
        [[nodiscard]] std::vector<std::vector<Point>> FilterPoints(const std::vector<Point>& points) const
        {
            std::vector<std::vector<Point>> groups;
            if (points.empty()) return groups;

            std::vector<Point> currentGroup;
            currentGroup.push_back(points[0]);

            for (size_t i = 1; i < points.size(); ++i) {
                double dist = Point::Distance(points[i-1], points[i]);
                if (dist <= _maxDistanceBetweenPoints) {
                    currentGroup.push_back(points[i]);
                    continue;
                }

                if (currentGroup.size() > 0) {
                    std::vector newGroup(currentGroup.begin(), currentGroup.end());
                    groups.emplace_back(newGroup);
                }

                currentGroup.clear();
                currentGroup.push_back(points[i]);
            }
            if (currentGroup.size() > 0) {
                std::vector newGroup(currentGroup.begin(), currentGroup.end());
                groups.emplace_back(newGroup);
            }

            if (groups.size() > 1)
            {
                auto first_group = groups[0];
                auto last_group = groups[groups.size() - 1];

                auto first_point = first_group.front();
                auto last_point = last_group.back();

                if (Point::Distance(first_point, last_point) <= _maxDistanceBetweenPoints)
                {
                    first_group.insert(first_group.end(), last_group.begin(), last_group.end());
                    groups[0] = first_group;
                    groups.pop_back();
                }
            }

            return groups;
        }
    };
}
