#pragma once

#include "Configurable.hpp"
#include <math.h>

namespace Manhattan::config {
struct WheelGeometry : Configurable {
    double radius;
    int32_t pulsesPerRotation;

    double circumference() const
    {
        return 2.0 * M_PI * radius;
    }

    double ticksToMeters(const int32_t ticks) const
    {
        return static_cast<double>(ticks) * circumference() / static_cast<double>(pulsesPerRotation);
    }

    void configure(const Config& config) override
    {
        radius = config["radius"].value<double>();
        pulsesPerRotation =  config["pulses_per_rotation"].value<int32_t>();
    }
};

struct DifferentialDriveGeometry : Configurable {
    WheelGeometry left;
    WheelGeometry right;
    double wheelBase;

    void configure(const Config& config) override
    {
        left.configure(config["left"]);
        right.configure(config["right"]);

        wheelBase = config["wheel_base"].value<double>();
    }
};

}