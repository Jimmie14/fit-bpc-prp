#include "LineEstimator.hpp"

#include <iostream>
#include <cmath>

using namespace std;

constexpr float LINE_THRESHOLD = .65;
constexpr double CONTINUOUS_SUM_EPSILON = 0.001;
constexpr double CONTINUOUS_EMA_ALPHA = 0.20;

LineEstimator::LineEstimator(const unsigned int maxIntensity, const unsigned int minIntensity) {
    _maxIntensity[0] = maxIntensity;
    _minIntensity[0] = minIntensity;

    _maxIntensity[1] = maxIntensity;
    _minIntensity[1] = minIntensity;
}

double LineEstimator::NormalizeValue(const unsigned int value, SensorLocation location)
{
    const auto index = static_cast<int>(location);

    if (_maxIntensity[index] < value)
        _maxIntensity[index] = value;

    if (_minIntensity[index] > value)
        _minIntensity[index] = value;

    const auto range = _maxIntensity[index] - _minIntensity[index];
    if (range < 1.0) return 0.0;

    const auto normalized = (static_cast<double>(value) - _minIntensity[index]) / range;
    return std::clamp(pow(normalized, .6), 0.0, 1.0);
}

DiscreteLinePose LineEstimator::EstimateDiscrete(const unsigned int leftVal, const unsigned int rightVal)
{
    const bool leftDetected = NormalizeValue(leftVal, SensorLocation::Left) > LINE_THRESHOLD;
    const bool rightDetected = NormalizeValue(rightVal, SensorLocation::Right) > LINE_THRESHOLD;

    if (leftDetected && rightDetected) {
        return DiscreteLinePose::LineBoth;
    }

    if (leftDetected) {
        return DiscreteLinePose::LineOnLeft;
    }

    if (rightDetected) {
        return DiscreteLinePose::LineOnRight;
    }

    return DiscreteLinePose::LineNone;
}

double LineEstimator::EstimateContinuousLinePose(const unsigned int leftValue, const unsigned int rightValue) {
    const double left = NormalizeValue(leftValue, SensorLocation::Left);
    const double right = NormalizeValue(rightValue, SensorLocation::Right);

    double sum = left + right;

    const auto leftDetected = left > LINE_THRESHOLD;
    const auto rightDetected = right > LINE_THRESHOLD;
    const auto hasLine = leftDetected || rightDetected;

    double linePose = _lastContinuousPose;
    if (hasLine && sum >= CONTINUOUS_SUM_EPSILON) {
        linePose = std::clamp((right - left) / sum, -1.0, 1.0);
        _lastContinuousPose = linePose;

        if (leftDetected && rightDetected) _lastDiscretePose = DiscreteLinePose::LineBoth;
        else if (leftDetected) _lastDiscretePose = DiscreteLinePose::LineOnLeft;
        else _lastDiscretePose = DiscreteLinePose::LineOnRight;

    } else {
        auto fallback = _lastContinuousPose;
        switch (_lastDiscretePose) {
            case DiscreteLinePose::LineOnLeft:
                fallback = -1;
                break;
            case DiscreteLinePose::LineOnRight:
                fallback = +1;
                break;
            case DiscreteLinePose::LineBoth:
                fallback = 0.0;
                break;
            case DiscreteLinePose::LineNone:
                fallback = _lastContinuousPose;
                break;
        }

        linePose = fallback;
    }

    const auto alpha = hasLine ? 0.25 : 0.08;

    if (!_hasContinuousEma) {
        _continuousEma = linePose;
        _hasContinuousEma = true;
    } else {
        _continuousEma = alpha * linePose + (1.0 - alpha) * _continuousEma;
    }

    return std::clamp(_continuousEma, -1.0, 1.0);
}
