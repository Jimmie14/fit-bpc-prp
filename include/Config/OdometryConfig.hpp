#pragma once

#include "Configurable.hpp"

namespace Manhattan::config
{
struct OdometryConfig : Configurable {
    std::string odometryTopic;
    double linearVelocitySmoothingTime;
    double angularVelocitySmoothingTime;

    void configure(const Config& config) override
    {
        odometryTopic = config["odometry_topic"].value<std::string>();
        linearVelocitySmoothingTime = config["linear_velocity_smoothing_time"].value<double>();
        angularVelocitySmoothingTime = config["angular_velocity_smoothing_time"].value<double>();
    }
};

}