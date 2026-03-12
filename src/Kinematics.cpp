#include "Kinematics.hpp"
#include "cmath"

Kinematics::Kinematics(double wheelRadius, double wheelBase, int ticksRevolution)
    : _wheelRadius(wheelRadius), _wheelBase(wheelBase), _ticksRevolution(ticksRevolution)
{

}

RobotSpeed Kinematics::forward(WheelSpeed speed) const {
    RobotSpeed result = { };

    result.linear = (_wheelRadius / 2) * (speed.right + speed.left);
    result.angular = (_wheelRadius / _wheelBase) * (speed.right - speed.left);

    return result;
}

WheelSpeed Kinematics::inverse(RobotSpeed speed) const {
    WheelSpeed result = { };

    result.right = (2 * speed.linear + speed.angular * _wheelBase) / (2 * _wheelRadius);
    result.left = (2 * speed.linear - speed.angular * _wheelBase) / (2 * _wheelRadius);

    return result;
}

Coord Kinematics::forward(Encoders encoders) const {
    const auto distanceLeft = (double)encoders.left / (double)_ticksRevolution * _wheelRadius;
    const auto distanceRight = (double)encoders.right / (double)_ticksRevolution * _wheelRadius;

    const auto deltaLinear = (distanceRight + distanceLeft) / 2;
    const auto deltaAngular = (distanceRight - distanceLeft) / _wheelBase;

    Coord result = { };

    result.x = deltaLinear * cos(deltaAngular * 0.5);
    result.y = deltaLinear * sin(deltaAngular * 0.5);

    return { };
}

Encoders Kinematics::inverse(Coord coord) const {
    return { };
}

