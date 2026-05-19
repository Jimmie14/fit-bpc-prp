#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "Vector2.hpp"

namespace Manhattan::math {

class LinearPath {
public:
    struct ClosestPointResult {
        Vector2 position;
        double distanceAlongPath {};
        double distanceToPathSq {};
    };

    void init(const std::vector<Vector2>& points)
    {
        _waypoints = points;
        _segmentLengths.clear();
        _totalLength = 0.0;

        if (_waypoints.size() < 2)
            return;

        _segmentLengths.resize(_waypoints.size() - 1);

        for (size_t i = 0; i < _waypoints.size() - 1; i++) {
            double len = Vector2::distance(_waypoints[i], _waypoints[i + 1]);
            _segmentLengths[i] = len;
            _totalLength += len;
        }
    }

    [[nodiscard]] bool HasPath() const
    {
        return _waypoints.size() >= 2;
    }

    [[nodiscard]] double getTotalLength() const
    {
        return _totalLength;
    }

    [[nodiscard]] Vector2 GetPointAtDistance(double distance) const
    {
        if (!HasPath())
            return Vector2 {0, 0};

        distance = std::max(0.0, std::min(_totalLength, distance));

        double accumulated = 0.0;

        for (size_t i = 0; i < _segmentLengths.size(); i++) {
            double segLen = _segmentLengths[i];

            if (distance <= accumulated + segLen || i == _segmentLengths.size() - 1) {
                double t = segLen > 0 ? (distance - accumulated) / segLen : 0.0;
                return Vector2::lerp(_waypoints[i], _waypoints[i + 1], t);
            }

            accumulated += segLen;
        }

        return _waypoints.back();
    }

    [[nodiscard]] ClosestPointResult findClosestPoint(const Vector2& p) const
    {
        ClosestPointResult result;
        result.distanceToPathSq = std::numeric_limits<double>::max();

        if (!HasPath())
            return result;

        double accumulated = 0.0;

        for (size_t i = 0; i < _waypoints.size() - 1; i++) {
            Vector2 a = _waypoints[i];
            Vector2 b = _waypoints[i + 1];
            Vector2 ab = b - a;

            double abLenSq = ab.sqrMagnitude();
            double t = 0.0;

            if (abLenSq > 1e-8)
                t = Vector2::dot(p - a, ab) / abLenSq;

            t = std::max(0.0, std::min(1.0, t));

            Vector2 pt = Vector2::lerp(a, b, t);
            double distSq = (p - pt).sqrMagnitude();

            if (distSq < result.distanceToPathSq) {
                result.distanceToPathSq = distSq;
                result.position = pt;
                result.distanceAlongPath = accumulated + _segmentLengths[i] * t;
            }

            accumulated += _segmentLengths[i];
        }

        return result;
    }

    [[nodiscard]] const std::vector<Vector2>& waypoints() const
    {
        return _waypoints;
    }

    [[nodiscard]] std::vector<Vector2> getWaypoints() const
    {
        return _waypoints;
    }

private:
    std::vector<Vector2> _waypoints;
    std::vector<double> _segmentLengths;
    double _totalLength = 0.0;
};

} // namespace Manhattan::Core