#include "Kinematics/Kinematics.hpp"

#include <cmath>

using namespace Manhattan::config;
using namespace Manhattan::kinematics;

namespace Manhattan::kinematics {
DifferentialDriveKinematics::DifferentialDriveKinematics(const DifferentialDriveGeometry& geometry)
{
    _geometry = geometry;
}

WheelAngularVelocity DifferentialDriveKinematics::inverse(const Twist& twist) const
{
    auto result = WheelAngularVelocity();

    const auto leftLinear = twist.linear + 0.5 * twist.angular * _geometry.wheelBase;
    const auto rightLinear = twist.linear - 0.5 * twist.angular * _geometry.wheelBase;

    result.right = leftLinear / _geometry.left.radius;
    result.left = rightLinear / _geometry.right.radius;

    return result;
}

Twist DifferentialDriveKinematics::forward(const WheelAngularVelocity& velocity) const
{
    const auto linear = angularToLinear(velocity);

    auto result = Twist();

    result.linear = 0.5 * (linear.left + linear.right);
    result.angular = (linear.right - linear.left) / _geometry.wheelBase;

    return result;
}

WheelLinearVelocity DifferentialDriveKinematics::ticksToLinear(const int32_t left, const int32_t right) const
{
    auto result = WheelLinearVelocity();

    result.left = _geometry.left.ticksToMeters(left);
    result.right = _geometry.right.ticksToMeters(right);

    return result;
}

WheelLinearVelocity DifferentialDriveKinematics::angularToLinear(const WheelAngularVelocity& angular) const
{
    auto result = WheelLinearVelocity();

    result.left = angular.left * _geometry.left.radius;
    result.right = angular.right * _geometry.right.radius;

    return result;
}

WheelAngularVelocity DifferentialDriveKinematics::linearToAngular(const WheelLinearVelocity& linear) const
{
    auto result = WheelAngularVelocity();

    result.left = linear.left / _geometry.left.radius;
    result.right = linear.right / _geometry.right.radius;

    return result;
}

DifferentialDriveOdometry::DifferentialDriveOdometry(const DifferentialDriveGeometry& geometry)
    : _wheelBase(geometry.wheelBase)
{

}

Pose DifferentialDriveOdometry::integrate(Pose pose, const WheelLinearVelocity& delta) const
{
    const double dPos = (delta.left + delta.right) / 2.0;
    const double dTheta = (delta.right - delta.left) / _wheelBase;

    const double heading = pose.theta;

    pose.position.x += dPos * std::sin(heading + dTheta / 2.0);
    pose.position.y += dPos * std::cos(heading + dTheta / 2.0);

    pose.theta += dTheta;

    return pose;
}

}