#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "Vector2.hpp"

namespace Manhattan::Core {
class SplineMath {
public:
    static Vector2 EvaluateBezier(const std::vector<Vector2>& ctrl, double t)
    {
        int n = static_cast<int>(ctrl.size()) - 1;
        if (n < 0)
            return Vector2 { 0, 0 };

        std::vector<Vector2> temp = ctrl;
        for (int r = 1; r <= n; r++) {
            for (int i = 0; i <= n - r; i++) {
                temp[i] = Vector2::Lerp(temp[i], temp[i + 1], t);
            }
        }
        return temp[0];
    }

    static Vector2 EvaluateDerivative(const std::vector<Vector2>& ctrl, double t)
    {
        int n = static_cast<int>(ctrl.size()) - 1;
        if (n <= 0)
            return Vector2 { 0, 0 };
        std::vector<Vector2> d(n);
        for (int i = 0; i < n; i++)
            d[i] = (ctrl[i + 1] - ctrl[i]) * n;
        return EvaluateBezier(d, t);
    }

    static Vector2 EvaluateSecondDerivative(const std::vector<Vector2>& ctrl, double t)
    {
        int n = static_cast<int>(ctrl.size()) - 1;
        if (n <= 1)
            return Vector2 { 0, 0 };
        std::vector<Vector2> dd(n - 1);
        for (int i = 0; i < n - 1; i++)
            dd[i] = (ctrl[i + 2] - ctrl[i + 1] * 2.0f + ctrl[i]) * (n * (n - 1));
        return EvaluateBezier(dd, t);
    }

    static double ArcLengthRecursive(const std::vector<Vector2>& ctrl, double t0, double t1, Vector2 p0, Vector2 p1, double tol)
    {
        auto tm = (t0 + t1) * 0.5f;
        Vector2 pm = EvaluateBezier(ctrl, tm);
        auto chord = Vector2::Distance(p0, p1);
        auto net = Vector2::Distance(p0, pm) + Vector2::Distance(pm, p1);

        if (net - chord <= tol)
            return (net + chord) * 0.5f;
        return ArcLengthRecursive(ctrl, t0, tm, p0, pm, tol) + ArcLengthRecursive(ctrl, tm, t1, pm, p1, tol);
    }

    static double ArcLength(const std::vector<Vector2>& ctrl, double tol)
    {
        return ArcLengthRecursive(ctrl, 0, 1, EvaluateBezier(ctrl, 0), EvaluateBezier(ctrl, 1), tol);
    }

    static double FindClosestParameterOnBezier(const std::vector<Vector2>& ctrl, Vector2 p)
    {
        double bestT = 0.0f;
        double bestDistSq = std::numeric_limits<double>::max();
        const int samples = 8;

        for (int i = 0; i < samples; i++) {
            double t = static_cast<double>(i) / (samples - 1);
            for (int iter = 0; iter < 10; iter++) {
                Vector2 b = EvaluateBezier(ctrl, t);
                Vector2 bd = EvaluateDerivative(ctrl, t);
                Vector2 bdd = EvaluateSecondDerivative(ctrl, t);

                double f = Vector2::Dot(b - p, bd);
                double fd = Vector2::Dot(bd, bd) + Vector2::Dot(b - p, bdd);

                if (std::abs(fd) < 1e-6)
                    break;
                double dt = -f / fd;
                t = std::max(0.0, std::min(1.0, t + dt));
                if (std::abs(dt) < 1e-6)
                    break;
            }

            double dSq = (EvaluateBezier(ctrl, t) - p).SqrMagnitude();
            if (dSq < bestDistSq) {
                bestDistSq = dSq;
                bestT = t;
            }
        }
        return bestT;
    }
};

class SplinePath {
public:
    struct BezierSegment {
        std::vector<Vector2> Ctrl; // 4 control points
        double Length;
        double StartAccumulatedLength;

        [[nodiscard]] Vector2 Evaluate(double t) const { return SplineMath::EvaluateBezier(Ctrl, t); }
    };

    struct ClosestPointResult {
        Vector2 Position;
        double DistanceAlongPath {};
        double DistanceToPathSq {};
    };

    [[nodiscard]] bool HasPath() const { return !segments.empty(); }

    void Initialize(const std::vector<Vector2>& waypoints)
    {
        segments.clear();
        totalLength = 0.0f;

        if (waypoints.size() < 2)
            return;

        std::vector<Vector2> bSplinePoints;
        bSplinePoints.push_back(waypoints.front());
        bSplinePoints.push_back(waypoints.front());
        bSplinePoints.insert(bSplinePoints.end(), waypoints.begin(), waypoints.end());
        bSplinePoints.push_back(waypoints.back());
        bSplinePoints.push_back(waypoints.back());

        double currentAccumulated = 0.0f;
        for (size_t i = 1; i < bSplinePoints.size() - 2; i++) {
            Vector2 p0 = bSplinePoints[i - 1];
            Vector2 p1 = bSplinePoints[i];
            Vector2 p2 = bSplinePoints[i + 1];
            Vector2 p3 = bSplinePoints[i + 2];

            std::vector segCtrl = {
                (p0 + p1 * 4.0 + p2) / 6.0,
                (p1 * 2.0 + p2) / 3.0,
                (p1 + p2 * 2.0) / 3.0,
                (p1 + p2 * 4.0 + p3) / 6.0
            };

            double len = SplineMath::ArcLength(segCtrl, 0.001f);
            segments.push_back({ segCtrl, len, currentAccumulated });
            currentAccumulated += len;
        }

        totalLength = currentAccumulated;
    }

    [[nodiscard]] Vector2 GetPointAtDistance(double distance) const
    {
        if (segments.empty())
            return Vector2 { 0, 0 };

        distance = std::max(0.0, std::min(totalLength, distance));

        for (size_t i = 0; i < segments.size(); i++) {
            const auto& seg = segments[i];
            if (distance <= seg.StartAccumulatedLength + seg.Length || i == segments.size() - 1) {
                double distInSeg = distance - seg.StartAccumulatedLength;
                double t = seg.Length > 0 ? distInSeg / seg.Length : 0;
                return seg.Evaluate(t);
            }
        }
        return segments.back().Ctrl[3];
    }

    [[nodiscard]] ClosestPointResult FindClosestPoint(Vector2 worldPos) const
    {
        ClosestPointResult result;
        result.DistanceToPathSq = std::numeric_limits<double>::max();

        if (segments.empty())
            return result;

        for (const auto& seg : segments) {
            double segT = SplineMath::FindClosestParameterOnBezier(seg.Ctrl, worldPos);
            Vector2 pt = SplineMath::EvaluateBezier(seg.Ctrl, segT);
            double distSq = (worldPos - pt).SqrMagnitude();

            if (distSq < result.DistanceToPathSq) {
                result.DistanceToPathSq = distSq;
                result.Position = pt;
                result.DistanceAlongPath = seg.StartAccumulatedLength + SplineMath::ArcLengthRecursive(seg.Ctrl, 0, segT, seg.Ctrl[0], pt, 0.001f);
            }
        }

        return result;
    }

    [[nodiscard]] double GetTotalLength() const { return totalLength; }

    [[nodiscard]] std::vector<BezierSegment> const& GetSegments() const { return segments; }

private:
    std::vector<BezierSegment> segments;
    double totalLength = 0.0f;
};
}