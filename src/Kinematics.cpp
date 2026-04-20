#include "Kinematics.hpp"

#include <cmath>
#include <numbers>

Kinematics::Kinematics(double wheelRadius, double wheelBase, int32_t pulsesPerRotation)
    : _wheelRadius(wheelRadius)
    , _wheelBase(wheelBase)
    , _pulsesPerRotation(pulsesPerRotation)
{
}
WheelSpeed Kinematics::inverse(RobotSpeed speed) const
{
    WheelSpeed result = {};

    result.right = (2 * speed.linear + speed.angular * _wheelBase) / (2 * _wheelRadius);
    result.left = (2 * speed.linear - speed.angular * _wheelBase) / (2 * _wheelRadius);

    return result;
}

RobotSpeed Kinematics::forward(WheelSpeed speed) const
{
    RobotSpeed result = {};

    result.linear = (_wheelRadius / 2) * (speed.right + speed.left);
    result.angular = (_wheelRadius / _wheelBase) * (speed.right - speed.left);

    return result;
}

double Kinematics::ticksToMeters(int32_t deltaTicks) const
{
    return static_cast<double>(deltaTicks) * 2.0 * M_PI * _wheelRadius / static_cast<double>(_pulsesPerRotation);
}

Pose2D Kinematics::integrate(Pose2D pose, double leftLinear, double rightLinear) const
{
    const double dPos = (leftLinear + rightLinear) / 2.0;
    const double dTheta = (rightLinear - leftLinear) / _wheelBase;

    pose.x += dPos * std::cos(pose.theta + dTheta / 2.0);
    pose.y += dPos * std::sin(pose.theta + dTheta / 2.0);
    pose.theta += dTheta;

    return pose;
}
