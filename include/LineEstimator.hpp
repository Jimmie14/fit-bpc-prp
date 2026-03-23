#pragma once

#include <ostream>

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

inline std::ostream& operator<<(std::ostream& os, DiscreteLinePose pose) {
    switch (pose) {
    case DiscreteLinePose::LineBoth:    os << "LineBoth"; break;
    case DiscreteLinePose::LineOnRight: os << "LineOnRight"; break;
    case DiscreteLinePose::LineOnLeft:  os << "LineOnLeft"; break;
    case DiscreteLinePose::LineNone:    os << "LineNone"; break;
    default:                            os << "Unknown"; break;
    }
    return os;
}

enum class SensorLocation
{
    Left = 0,
    Right = 1,
};

class LineEstimator {
    unsigned int _maxIntensity[2];
    unsigned int _minIntensity[2];
public:
    LineEstimator(unsigned int maxIntensity, unsigned int minIntensity);

    DiscreteLinePose EstimateDiscrete(unsigned int leftVal, unsigned int rightVal);

    float EstimateContinuousLinePose(unsigned int leftValue, unsigned int rightValue);

private:
    float NormalizeValue(unsigned int value, SensorLocation location);
};