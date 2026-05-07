#pragma once

#include "Math/Pose.hpp"

namespace Manhattan::Core {

struct WheelSpeed {
    double left = 0.0; // rad/s
    double right = 0.0; // rad/s
};

class Kinematics {
public:
    Kinematics(double wheelRadius, double wheelBase, int32_t pulsesPerRotation);

    [[nodiscard]] WheelSpeed inverse(const Twist& twist) const;

    [[nodiscard]] Twist forward(WheelSpeed speed) const;

    [[nodiscard]] double ticksToMeters(int32_t deltaTicks) const;

    [[nodiscard]] Pose integrate(Pose pose, double leftLinear, double rightLinear) const;

    [[nodiscard]] double wheelRadius() const
    {
        return _wheelRadius;
    }
    [[nodiscard]] double wheelBase() const
    {
        return _wheelBase;
    }
    [[nodiscard]] int32_t pulsesPerRotation() const
    {
        return _pulsesPerRotation;
    }

private:
    double _wheelRadius;
    double _wheelBase;
    int32_t _pulsesPerRotation;
};

}