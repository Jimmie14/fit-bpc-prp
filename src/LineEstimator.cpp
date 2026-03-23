#include "LineEstimator.hpp"

#include <iostream>

using namespace std;

constexpr float LINE_THRESHOLD = .5;
constexpr float ALPHA = 0.01f;

LineEstimator::LineEstimator(const unsigned int maxIntensity, const unsigned int minIntensity) {
    _maxIntensity[0] = maxIntensity;
    _minIntensity[0] = minIntensity;

    _maxIntensity[1] = maxIntensity;
    _minIntensity[1] = minIntensity;
}

float LineEstimator::NormalizeValue(const unsigned int value, SensorLocation location)
{
    const auto index = static_cast<int>(location);

    if (value > _maxIntensity[index])
        _maxIntensity[index] = ALPHA * value + (1 - ALPHA) * _maxIntensity[index];
    else
        _maxIntensity[index] = (1 - ALPHA) * _maxIntensity[index];

    if (value < _minIntensity[index])
        _minIntensity[index] = ALPHA * value + (1 - ALPHA) * _minIntensity[index];
    else
        _minIntensity[index] = (1 - ALPHA) * _minIntensity[index];

    return (value - _minIntensity[index]) / static_cast<double>(_maxIntensity[index] - _minIntensity[index]);
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

float LineEstimator::EstimateContinuousLinePose(const unsigned int leftValue, const unsigned int rightValue) {
    // Calculate a normalized error value between -1.0 and 1.0
    // -1.0 implies line is fully on the left
    //  1.0 implies line is fully on the right
    //  0.0 implies centered or no signal

    double sum = leftValue + rightValue;
    if (sum < 0.001f) return 0.0f;

    // Normalized difference formula
    return static_cast<double>(leftValue - rightValue) / sum;
}
