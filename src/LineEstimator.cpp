#include "LineEstimator.hpp"

#include <cstdint>

using namespace std;

constexpr uint16_t LINE_THRESHOLD = 100;

DiscreteLinePose LineEstimator::EstimateDiscrete(unsigned int leftVal, unsigned int rightVal) {
    // Assuming higher values indicate line presence (e.g., black line, reflective sensor)
    bool leftDetected = leftVal > LINE_THRESHOLD;
    bool rightDetected = rightVal > LINE_THRESHOLD;

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

float LineEstimator::EstimateContinuousLinePose(float left_value, float right_value) {
    // Calculate a normalized error value between -1.0 and 1.0
    // -1.0 implies line is fully on the left
    //  1.0 implies line is fully on the right
    //  0.0 implies centered or no signal

    float sum = left_value + right_value;

    // specific check to avoid division by zero
    if (sum < 0.001f) {
        return 0.0f;
    }

    // Normalized difference formula
    return (right_value - left_value) / sum;
}
