#pragma once
#include "OccupancyGrid.hpp"
#include "Vector2.hpp"

namespace Manhattan::Core
{
    struct Pose
    {
        Vector2 Position;
        double Rotation;
    };

    class PoseMatcher
    {
        const OccupancyGrid &_grid;
        int _numIterations;

    public:
        explicit PoseMatcher(const OccupancyGrid &grid, const int numIterations = 5)
            : _grid(grid),
              _numIterations(numIterations)
        { }

        Pose Match(const std::vector<Vector2>& scanPoints, Vector2 estimatedPos, double estimatedRot) const
        {
            Vector2 pos = estimatedPos;
            double rot = estimatedRot;

            for (int iter = 0; iter < _numIterations; iter++)
            {
                // Initialize 3x3 Hessian matrix and 3x1 boundary vector with zeros
                double h[3][3] = { {} };
                double b[3] = {};

                double cos_val = std::cos(rot);
                double sin_val = std::sin(rot);

                for (const auto& p : scanPoints)
                {
                    // Note: Change p.x and p.y if your Point struct uses uppercase X and Y
                    double tx = p.x * cos_val - p.y * sin_val;
                    double ty = p.x * sin_val + p.y * cos_val;
                    auto world = Vector2(pos.x + tx, pos.y + ty);

                    auto cellOffset = GridOffsetForInterpolation(world);
                    int x0 = std::get<0>(cellOffset);
                    int y0 = std::get<1>(cellOffset);
                    double fx = std::get<2>(cellOffset);
                    double fy = std::get<3>(cellOffset);

                    int x1 = x0 + 1;
                    int y1 = y0 + 1;

                    if (!_grid.InBounds(x0, y0) || !_grid.InBounds(x1, y1))
                        continue;

                    double p00 = _grid.GetProbability(x0, y0);
                    double p10 = _grid.GetProbability(x1, y0);
                    double p01 = _grid.GetProbability(x0, y1);
                    double p11 = _grid.GetProbability(x1, y1);

                    if (p00 <= 0.5 && p10 <= 0.5 && p01 <= 0.5 && p11 <= 0.5)
                        continue;

                    double m = (p00 * (1.0 - fx) * (1.0 - fy)) +
                               (p10 * fx * (1.0 - fy)) +
                               (p01 * (1.0 - fx) * fy) +
                               (p11 * fx * fy);

                    double dx = ((p10 - p00) * (1.0 - fy) + (p11 - p01) * fy) / _grid.GetCellSize();
                    double dy = ((p01 - p00) * (1.0 - fx) + (p11 - p10) * fx) / _grid.GetCellSize();

                    double jThetaX = -ty;
                    double jThetaY = tx;

                    double gX = dx;
                    double gY = dy;
                    double gTheta = dx * jThetaX + dy * jThetaY;

                    double error = 1.0 - m;

                    h[0][0] += gX * gX; h[0][1] += gX * gY; h[0][2] += gX * gTheta;
                    h[1][0] += gY * gX; h[1][1] += gY * gY; h[1][2] += gY * gTheta;
                    h[2][0] += gTheta * gX; h[2][1] += gTheta * gY; h[2][2] += gTheta * gTheta;

                    b[0] += gX * error;
                    b[1] += gY * error;
                    b[2] += gTheta * error;
                }

                double det = h[0][0] * (h[1][1] * h[2][2] - h[1][2] * h[2][1])
                           - h[0][1] * (h[1][0] * h[2][2] - h[1][2] * h[2][0])
                           + h[0][2] * (h[1][0] * h[2][1] - h[1][1] * h[2][0]);

                double dPosMx = 0.0, dPosMy = 0.0, dRotM = 0.0;

                if (std::abs(det) > 1e-6)
                {
                    double invDet = 1.0 / det;

                    double i00 = (h[1][1] * h[2][2] - h[1][2] * h[2][1]) * invDet;
                    double i01 = (h[0][2] * h[2][1] - h[0][1] * h[2][2]) * invDet;
                    double i02 = (h[0][1] * h[1][2] - h[0][2] * h[1][1]) * invDet;

                    double i10 = (h[1][2] * h[2][0] - h[1][0] * h[2][2]) * invDet;
                    double i11 = (h[0][0] * h[2][2] - h[0][2] * h[2][0]) * invDet;
                    double i12 = (h[0][2] * h[1][0] - h[0][0] * h[1][2]) * invDet;

                    double i20 = (h[1][0] * h[2][1] - h[1][1] * h[2][0]) * invDet;
                    double i21 = (h[0][1] * h[2][0] - h[0][0] * h[2][1]) * invDet;
                    double i22 = (h[0][0] * h[1][1] - h[0][1] * h[1][0]) * invDet;

                    dPosMx = i00 * b[0] + i01 * b[1] + i02 * b[2];
                    dPosMy = i10 * b[0] + i11 * b[1] + i12 * b[2];
                    dRotM = i20 * b[0] + i21 * b[1] + i22 * b[2];
                }

                pos.x += std::clamp(dPosMx, -0.1, 0.1);
                pos.y += std::clamp(dPosMy, -0.1, 0.1);
                rot += std::clamp(dRotM, -0.05, 0.05);
            }

            return Pose { pos, rot };
        }

    private:
        std::tuple<int, int, double, double> GridOffsetForInterpolation(const Vector2& worldPos) const
        {
            // Subtract 0.5f so integer boundaries align with the exact centers of the cells
            double gridFloatX = (worldPos.x / _grid.GetCellSize()) + (_grid.GetWidth() * 0.5) - 0.5;
            double gridFloatY = (worldPos.y / _grid.GetCellSize()) + (_grid.GetHeight() * 0.5) - 0.5;

            int cellX = static_cast<int>(std::floor(gridFloatX));
            int cellY = static_cast<int>(std::floor(gridFloatY));

            double fractionX = gridFloatX - cellX;
            double fractionY = gridFloatY - cellY;

            return { cellX, cellY, fractionX, fractionY };
        }
    };
}
