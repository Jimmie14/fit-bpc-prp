#pragma once

#include "OccupancyGrid.hpp"
#include "Pose.hpp"
#include <iostream>

namespace Manhattan::Core {
struct PoseMatchResult {
    Pose pose { Vector2 {}, 0.0 };
    double confidence { 0.0 };

    [[nodiscard]] static PoseMatchResult Best(const PoseMatchResult& left, const PoseMatchResult& right)
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

        auto totalError = 0.0;
        auto knownCount = 0;
        auto usedPoints = 0;

        auto condSum = 0.0;
        auto validIterations = 0;
        auto lastDeltaNorm = 0.0;

        for (int iter = 0; iter < _numIterations; iter++) {
            // Reset per-iteration accumulators so final values reflect the converged pose
            totalError = 0.0;
            knownCount = 0;
            usedPoints = 0;

            // Initialize 3x3 Hessian matrix and 3x1 boundary vector with zeros
            double h[3][3] = { {} };
            double b[3] = {};

            const auto cosVal = std::cos(rot);
            const auto sinVal = std::sin(rot);

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

                if (!_grid.InBounds(x0, y0) || !_grid.InBounds(x1, y1)) {
                    continue;
                }

                const auto p00 = _grid.GetProbability(x0, y0);
                const auto p10 = _grid.GetProbability(x1, y0);
                const auto p01 = _grid.GetProbability(x0, y1);
                const auto p11 = _grid.GetProbability(x1, y1);

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

                totalError += error * error;

                if (std::abs(m - 0.5) > 0.05) knownCount++;
                usedPoints++;
            }

            // Enforce hessian symmetry
            h[1][0] = h[0][1];
            h[2][0] = h[0][2];
            h[2][1] = h[1][2];

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

                const auto normH = sqrt(h[0][0] * h[0][0] + h[1][1] * h[1][1] + h[2][2] * h[2][2] + 2 * (h[0][1] * h[0][1] + h[0][2] * h[0][2] + h[1][2] * h[1][2]));
                const auto condApprox = normH / (abs(det) + 1e-9);

                condSum += condApprox;
                validIterations++;
            }

            pos.x += std::clamp(dPosMx, -0.1, 0.1);
            pos.y += std::clamp(dPosMy, -0.1, 0.1);
            rot += std::clamp(dRotM, -0.05, 0.05);

            lastDeltaNorm = std::sqrt(dPosMx*dPosMx + dPosMy*dPosMy + dRotM*dRotM);
        }

        // Fix 1: compute avgCond
        const double avgCond = validIterations > 0 ? condSum / validIterations : 1e9;

        constexpr double sigma2 = 0.25;
        constexpr double deltaSigma2 = 0.01;
        constexpr double condScale = 1e6;

        const double avgError = totalError / std::max(1, usedPoints);
        const double fitScore = std::exp(-avgError / sigma2);
        const double coverageScore = static_cast<double>(usedPoints) / std::max<size_t>(1, scanPoints.size());
        const double deltaScore = std::exp(-(lastDeltaNorm * lastDeltaNorm) / deltaSigma2);
        const double conditioningScore = 1.0 / (1.0 + avgCond / condScale);

        const double knownRatio = static_cast<double>(knownCount) / std::max(1, usedPoints);

        // Pose location penalty
        const auto poseCell = _grid.WorldToGrid(pos);
        const auto poseProbability = _grid.GetProbability(poseCell.x, poseCell.y);
        const double poseKnownScore = std::clamp(std::abs(poseProbability - 0.5) * 2.0, 0.0, 1.0);

        const double confidence =
            0.35 * fitScore +            // how well scan endpoints hit occupied cells
            0.20 * coverageScore +       // fraction of scan points that were in-bounds
            0.20 * knownRatio +          // fraction of points in decided (non-unknown) cells
            0.10 * deltaScore +          // convergence stability
            0.10 * conditioningScore +   // Hessian quality
            0.05 * poseKnownScore;       // is the pose itself in observed space?

        std::cout << "conf (raw)  : " << confidence << std::endl;

        return { Pose(pos, rot), std::clamp(confidence, 0.0, 1.0) };
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
