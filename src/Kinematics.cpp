#include "Kinematics.hpp"

#include <cmath>
#include <numbers>

namespace Manhattan::Core {
Kinematics::Kinematics(const double wheelRadius, const double wheelBase, const int32_t pulsesPerRotation)
    : _wheelRadius(wheelRadius)
    , _wheelBase(wheelBase)
    , _pulsesPerRotation(pulsesPerRotation)
{
}
WheelSpeed Kinematics::inverse(const RobotSpeed speed) const
{
    WheelSpeed result = {};

    result.right = (2 * speed.linear + speed.angular * _wheelBase) / (2 * _wheelRadius);
    result.left = (2 * speed.linear - speed.angular * _wheelBase) / (2 * _wheelRadius);

    return result;
}

RobotSpeed Kinematics::forward(const WheelSpeed speed) const
{
    RobotSpeed result = {};

    result.linear = (_wheelRadius / 2) * (speed.right + speed.left);
    result.angular = (_wheelRadius / _wheelBase) * (speed.right - speed.left);

    return result;
}

double Kinematics::ticksToMeters(const int32_t deltaTicks) const
{
    return static_cast<double>(deltaTicks) * 2.0 * M_PI * _wheelRadius / static_cast<double>(_pulsesPerRotation);
}

Pose Kinematics::integrate(Pose pose, const double leftLinear, const double rightLinear) const
{
    const double dPos = (leftLinear + rightLinear) / 2.0;
    const double dTheta = (rightLinear - leftLinear) / _wheelBase;

    pose.position.x += dPos * std::cos(pose.rotation + dTheta / 2.0);
    pose.position.y += dPos * std::sin(pose.rotation + dTheta / 2.0);
    pose.rotation += dTheta;

    return pose;
}
}