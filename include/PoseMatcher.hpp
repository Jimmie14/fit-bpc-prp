#pragma once
#include "OccupancyGrid.hpp"
#include "Vector2.hpp"

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

    [[nodiscard]] std::string ToString() const
    {
        return "(pos=" + position.ToString() + ", theta=" + std::to_string(rotation) + ")";
    }
};

struct PoseMatchResult {
    Pose pose { Vector2 {}, 0.0 };
    double confidence { 0.0 };

    [[nodiscard]] static PoseMatchResult Combine(const PoseMatchResult& left, const PoseMatchResult& right)
    {
        return left.confidence > right.confidence ? left : right;
    }
};

class PoseMatcher {
public:
    // Gauss-Newton Optimization
    explicit PoseMatcher(const OccupancyGrid& grid, const int numIterations = 5)
        : _grid(grid)
        , _numIterations(numIterations)
    {
    }

    [[nodiscard]] PoseMatchResult Match(const std::vector<Vector2>& scanPoints, const Pose& estimatedPose) const
    {
        auto pos = estimatedPose.position;
        auto rot = estimatedPose.rotation;
        const auto pointsCount = static_cast<double>(scanPoints.size());

        auto totalProbability = 0.0;
        auto totalObservability = 0.0;

        for (int iter = 0; iter < _numIterations; iter++) {
            totalProbability = 0.0;

            // Initialize 3x3 Hessian matrix and 3x1 boundary vector with zeros
            double h[3][3] = { {} };
            double b[3] = {};

            auto cosVal = std::cos(rot);
            auto sinVal = std::sin(rot);

            for (const auto& p : scanPoints) {
                const auto tx = p.x * cosVal - p.y * sinVal;
                const auto ty = p.x * sinVal + p.y * cosVal;
                auto world = Vector2(pos.x + tx, pos.y + ty);

                auto cellOffset = GridOffsetForInterpolation(world);
                const auto x0 = std::get<0>(cellOffset);
                const auto y0 = std::get<1>(cellOffset);
                const auto fx = std::get<2>(cellOffset);
                const auto fy = std::get<3>(cellOffset);

                const auto x1 = x0 + 1;
                const auto y1 = y0 + 1;

                if (!_grid.InBounds(x0, y0) || !_grid.InBounds(x1, y1))
                    continue;

                const auto p00 = _grid.GetProbability(x0, y0);
                const auto p10 = _grid.GetProbability(x1, y0);
                const auto p01 = _grid.GetProbability(x0, y1);
                const auto p11 = _grid.GetProbability(x1, y1);

                // if (p00 <= 0.5 && p10 <= 0.5 && p01 <= 0.5 && p11 <= 0.5)
                //     continue;

                const auto m = (p00 * (1.0 - fx) * (1.0 - fy)) + (p10 * fx * (1.0 - fy)) + (p01 * (1.0 - fx) * fy) + (p11 * fx * fy);

                const auto dx = ((p10 - p00) * (1.0 - fy) + (p11 - p01) * fy) / _grid.GetCellSize();
                const auto dy = ((p01 - p00) * (1.0 - fx) + (p11 - p10) * fx) / _grid.GetCellSize();

                const auto jThetaX = -ty;
                const auto jThetaY = tx;

                const auto gX = dx;
                const auto gY = dy;
                const auto gTheta = dx * jThetaX + dy * jThetaY;

                const auto error = 1.0 - m;

                h[0][0] += gX * gX;
                h[0][1] += gX * gY;
                h[0][2] += gX * gTheta;
                h[1][0] += gY * gX;
                h[1][1] += gY * gY;
                h[1][2] += gY * gTheta;
                h[2][0] += gTheta * gX;
                h[2][1] += gTheta * gY;
                h[2][2] += gTheta * gTheta;

                b[0] += gX * error;
                b[1] += gY * error;
                b[2] += gTheta * error;

                totalProbability += m;
            }

            const auto det = h[0][0] * (h[1][1] * h[2][2] - h[1][2] * h[2][1]) - h[0][1] * (h[1][0] * h[2][2] - h[1][2] * h[2][0]) + h[0][2] * (h[1][0] * h[2][1] - h[1][1] * h[2][0]);

            auto dPosMx = 0.0, dPosMy = 0.0, dRotM = 0.0;

            if (std::abs(det) > 1e-6) {
                const auto invDet = 1.0 / det;

                const auto i00 = (h[1][1] * h[2][2] - h[1][2] * h[2][1]) * invDet;
                const auto i01 = (h[0][2] * h[2][1] - h[0][1] * h[2][2]) * invDet;
                const auto i02 = (h[0][1] * h[1][2] - h[0][2] * h[1][1]) * invDet;

                const auto i10 = (h[1][2] * h[2][0] - h[1][0] * h[2][2]) * invDet;
                const auto i11 = (h[0][0] * h[2][2] - h[0][2] * h[2][0]) * invDet;
                const auto i12 = (h[0][2] * h[1][0] - h[0][0] * h[1][2]) * invDet;

                const auto i20 = (h[1][0] * h[2][1] - h[1][1] * h[2][0]) * invDet;
                const auto i21 = (h[0][1] * h[2][0] - h[0][0] * h[2][1]) * invDet;
                const auto i22 = (h[0][0] * h[1][1] - h[0][1] * h[1][0]) * invDet;

                dPosMx = i00 * b[0] + i01 * b[1] + i02 * b[2];
                dPosMy = i10 * b[0] + i11 * b[1] + i12 * b[2];
                dRotM = i20 * b[0] + i21 * b[1] + i22 * b[2];

                const auto quality = std::log(std::abs(det) + 1.0);
                const auto qualityNorm = quality / (quality + 1.0);

                const auto trace = h[0][0] + h[1][1] + h[2][2];
                const auto traceAvg = trace / pointsCount;
                const auto traceNorm = traceAvg / (traceAvg + 1.0); // todo: calculate k over sliding mediat window

                // weighted average of determinant quality and constraint magnitude
                totalObservability += (qualityNorm * 0.6 + traceNorm * 0.4) / static_cast<double>(_numIterations);
            }

            pos.x += std::clamp(dPosMx, -0.1, 0.1);
            pos.y += std::clamp(dPosMy, -0.1, 0.1);
            rot += std::clamp(dRotM, -0.05, 0.05);
        }

        const auto coverage = scanPoints.empty()
            ? 0.0
            : totalProbability / static_cast<double>(scanPoints.size());

        const auto confidence = std::clamp(coverage, 0.0, 1.0);

        return { Pose(pos, rot), confidence };
    }

private:
    const OccupancyGrid& _grid;
    int _numIterations;

    [[nodiscard]] std::tuple<int, int, double, double> GridOffsetForInterpolation(const Vector2& worldPos) const
    {
        // Subtract 0.5f so integer boundaries align with the exact centers of the cells
        const auto gridFloatX = (worldPos.x / _grid.GetCellSize()) + (_grid.GetWidth() * 0.5) - 0.5;
        const auto gridFloatY = (worldPos.y / _grid.GetCellSize()) + (_grid.GetHeight() * 0.5) - 0.5;

        const auto cellX = static_cast<int>(std::floor(gridFloatX));
        const auto cellY = static_cast<int>(std::floor(gridFloatY));

        const auto fractionX = gridFloatX - cellX;
        const auto fractionY = gridFloatY - cellY;

        return { cellX, cellY, fractionX, fractionY };
    }
};
} // namespace Manhattan::Core
