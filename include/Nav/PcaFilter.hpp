#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

class PcaFitter
{
public:
    struct FittedLine
    {
        Manhattan::math::Vector2 Point;      // Mean of the data
        Manhattan::math::Vector2 Direction;  // Principal direction
        Manhattan::math::Vector2 Normal;     // Perpendicular to direction
        float Variance;     // Variance along principal direction
        int PointCount;     // Number of fitted points
    };

    static FittedLine FitLine(const std::vector<Manhattan::math::Vector2>& points)
    {
        if (points.empty())
            throw std::runtime_error("Points list cannot be empty");

        // Calculate mean
        Manhattan::math::Vector2 mean(0.0f, 0.0f);

        for (const auto& point : points)
        {
            mean.x += point.x;
            mean.y += point.y;
        }

        mean.x /= static_cast<float>(points.size());
        mean.y /= static_cast<float>(points.size());

        // Calculate covariance matrix
        float cov00 = 0.0f;
        float cov01 = 0.0f;
        float cov11 = 0.0f;

        for (const auto& point : points)
        {
            Manhattan::math::Vector2 diff(point.x - mean.x, point.y - mean.y);

            cov00 += diff.x * diff.x;
            cov01 += diff.x * diff.y;
            cov11 += diff.y * diff.y;
        }

        const float count = static_cast<float>(points.size());

        cov00 /= count;
        cov01 /= count;
        cov11 /= count;

        // Eigenvalues for 2x2 symmetric matrix
        float trace = cov00 + cov11;
        float det = cov00 * cov11 - cov01 * cov01;

        float discriminant = trace * trace * 0.25f - det;
        discriminant = std::max(discriminant, 0.0f);

        float lambda1 = trace * 0.5f + std::sqrt(discriminant);

        // Principal eigenvector
        const Manhattan::math::Vector2 direction = GetEigenvector(cov00, cov01, cov11, lambda1);

        // Normal vector
        const Manhattan::math::Vector2 normal(-direction.y, direction.x);

        FittedLine result;
        result.Point = mean;
        result.Direction = direction;
        result.Normal = normal;
        result.Variance = lambda1;
        result.PointCount = static_cast<int>(points.size());

        return result;
    }

private:
    static Manhattan::math::Vector2 GetEigenvector(float a, float b, float d, float eigenvalue)
    {
        // For matrix:
        // [a b]
        // [b d]

        float x = b;
        float y = eigenvalue - a;

        float length = std::sqrt(x * x + y * y);

        if (length > 0.0001f)
            return Manhattan::math::Vector2(x / length, y / length);

        // Fallback
        x = eigenvalue - d;
        y = b;

        length = std::sqrt(x * x + y * y);

        if (length > 0.0001f)
            return Manhattan::math::Vector2(x / length, y / length);

        // Default fallback
        return Manhattan::math::Vector2(1.0f, 0.0f);
    }
};