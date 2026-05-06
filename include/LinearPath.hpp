#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "Math/Vector2.hpp"

namespace Manhattan::Core {

class LinearPath {
public:
    struct ClosestPointResult {
        Vector2 Position;
        double DistanceAlongPath {};
        double DistanceToPathSq {};
    };

    void Initialize(const std::vector<Vector2>& points)
    {
        waypoints = points;
        segmentLengths.clear();
        totalLength = 0.0;

        if (waypoints.size() < 2)
            return;

        segmentLengths.resize(waypoints.size() - 1);

        for (size_t i = 0; i < waypoints.size() - 1; i++) {
            double len = Vector2::Distance(waypoints[i], waypoints[i + 1]);
            segmentLengths[i] = len;
            totalLength += len;
        }
    }

    [[nodiscard]] bool HasPath() const
    {
        return waypoints.size() >= 2;
    }

    [[nodiscard]] double GetTotalLength() const
    {
        return totalLength;
    }

    [[nodiscard]] Vector2 GetPointAtDistance(double distance) const
    {
        if (!HasPath())
            return Vector2 {0, 0};

        distance = std::max(0.0, std::min(totalLength, distance));

        double accumulated = 0.0;

        for (size_t i = 0; i < segmentLengths.size(); i++) {
            double segLen = segmentLengths[i];

            if (distance <= accumulated + segLen || i == segmentLengths.size() - 1) {
                double t = segLen > 0 ? (distance - accumulated) / segLen : 0.0;
                return Vector2::Lerp(waypoints[i], waypoints[i + 1], t);
            }

            accumulated += segLen;
        }

        return waypoints.back();
    }

    [[nodiscard]] ClosestPointResult FindClosestPoint(const Vector2& p) const
    {
        ClosestPointResult result;
        result.DistanceToPathSq = std::numeric_limits<double>::max();

        if (!HasPath())
            return result;

        double accumulated = 0.0;

        for (size_t i = 0; i < waypoints.size() - 1; i++) {
            Vector2 a = waypoints[i];
            Vector2 b = waypoints[i + 1];
            Vector2 ab = b - a;

            double abLenSq = ab.SqrMagnitude();
            double t = 0.0;

            if (abLenSq > 1e-8)
                t = Vector2::Dot(p - a, ab) / abLenSq;

            t = std::max(0.0, std::min(1.0, t));

            Vector2 pt = Vector2::Lerp(a, b, t);
            double distSq = (p - pt).SqrMagnitude();

            if (distSq < result.DistanceToPathSq) {
                result.DistanceToPathSq = distSq;
                result.Position = pt;
                result.DistanceAlongPath = accumulated + segmentLengths[i] * t;
            }

            accumulated += segmentLengths[i];
        }

        return result;
    }

private:
    std::vector<Vector2> waypoints;
    std::vector<double> segmentLengths;
    double totalLength = 0.0;
};

} // namespace Manhattan::Core