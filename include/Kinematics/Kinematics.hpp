#pragma once

#include "Config/Kinematics.hpp"
#include "Math/Pose.hpp"

namespace Manhattan::kinematics {

using namespace Manhattan::math;

struct WheelAngularVelocity {
    double left;
    double right;

    WheelAngularVelocity() = default;

    WheelAngularVelocity(const double left, const double right)
        : left(left)
        , right(right)
    {
    }
};

struct WheelLinearVelocity {
    double left;
    double right;

    WheelLinearVelocity() = default;

    WheelLinearVelocity(const double left, const double right)
        : left(left)
        , right(right)
    {
    }
};

class DifferentialDriveKinematics {
public:
    explicit DifferentialDriveKinematics(const config::DifferentialDriveGeometry& geometry);

    [[nodiscard]] WheelAngularVelocity inverse(const Twist& twist) const;

    [[nodiscard]] Twist forward(const WheelAngularVelocity& velocity) const;

    [[nodiscard]] WheelLinearVelocity ticksToLinear(int32_t left, int32_t right) const;

    [[nodiscard]] WheelLinearVelocity angularToLinear(const WheelAngularVelocity& angular) const;

    [[nodiscard]] WheelAngularVelocity linearToAngular(const WheelLinearVelocity& linear) const;

    [[nodiscard]] config::DifferentialDriveGeometry geometry() const
    {
        return _geometry;
    }

private:
    config::DifferentialDriveGeometry _geometry;
};

class DifferentialDriveOdometry {
public:
    explicit DifferentialDriveOdometry(const config::DifferentialDriveGeometry& geometry);

    [[nodiscard]] Pose integrate(Pose pose, const WheelLinearVelocity& delta) const;

private:
    double _wheelBase;
};

}