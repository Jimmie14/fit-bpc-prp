#include "LineEstimator.hpp"

#include <iostream>

using namespace std;

constexpr float LINE_THRESHOLD = .5;
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

    const auto range = static_cast<double>(_maxIntensity[index] - _minIntensity[index]);
    if (range < 1.0) return 0.0;

    const auto normalized = (static_cast<double>(value) - static_cast<double>(_minIntensity[index])) / range;
    return std::clamp(normalized, 0.0, 1.0);
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

    double rawPose = 0.0;
    if (sum >= CONTINUOUS_SUM_EPSILON) {
        rawPose = (right - left) / sum;
        rawPose = std::clamp(rawPose, -1.0, 1.0);

        _lastContinuousPose = rawPose;
        _hasContinuousPose = true;
    } else if (_hasContinuousPose) {
        rawPose = _lastContinuousPose;
    }

    if (!_hasContinuousEma) {
        _continuousEma = rawPose;
        _hasContinuousEma = true;
    } else {
        _continuousEma = CONTINUOUS_EMA_ALPHA * rawPose + (1.0 - CONTINUOUS_EMA_ALPHA) * _continuousEma;
    }

    return std::clamp(_continuousEma, -1.0, 1.0);
}
