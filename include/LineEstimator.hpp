#pragma once

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

class LineEstimator {
public:
    LineEstimator();

    static DiscreteLinePose EstimateDiscrete(unsigned int leftVal, unsigned int rightVal);

    static float EstimateContinuousLinePose(float left_value, float right_value);
};