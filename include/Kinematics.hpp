#pragma once

#include <cstdint>

struct RobotSpeed {
    /**
     * Linear speed in m/s.
     */
    double linear = 0.0;

    /**
     * Angular speed in rad/s.
     */
    double angular = 0.0;
};

struct WheelSpeed {
    double left = 0.0; // rad/s
    double right = 0.0; // rad/s
};

struct Pose2D {
    double x = 0.0; // m
    double y = 0.0; // m
    double theta = 0.0; // rad
};

class Kinematics {
public:
    Kinematics(double wheelRadius, double wheelBase, int32_t pulsesPerRotation);

    [[nodiscard]] WheelSpeed inverse(RobotSpeed speed) const;

    [[nodiscard]] RobotSpeed forward(WheelSpeed speed) const;

    [[nodiscard]] double ticksToMeters(int32_t deltaTicks) const;

    [[nodiscard]] Pose2D integrate(Pose2D pose, double leftLinear, double rightLinear) const;

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
