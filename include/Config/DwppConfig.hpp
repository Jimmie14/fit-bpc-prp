#pragma once

#include "Configurable.hpp"

namespace Manhattan::config
{
struct DwppConfig : Configurable {
    double deltaTime;
    double lookaheadDistance;

    double linearAcceleration;
    double angularAcceleration;

    double maxLinearSpeed;
    double maxAngularSpeed;

    int linearVelocitySamples;
    int angularVelocitySamples;

    int simulationSteps;
    double simulationDeltaTime;

    double linearSpeedWeight;
    double angularSpeedWeight;

    double pathErrorWeight;
    double headingErrorWeight;
    double goalWeight;
    double speedWeight;

    void configure(const Config& config) override
    {
        deltaTime = config["delta_time"].value<double>();
        lookaheadDistance = config["lookahead_distance"].value<double>();

        linearAcceleration = config["linear_acceleration"].value<double>();
        angularAcceleration = config["angular_acceleration"].value<double>();

        maxLinearSpeed = config["max_linear_speed"].value<double>();
        maxAngularSpeed = config["max_angular_speed"].value<double>();

        linearVelocitySamples = config["linear_velocity_samples"].value<double>();
        angularVelocitySamples = config["angular_velocity_samples"].value<double>();

        simulationSteps = config["simulation_steps"].value<int>();
        simulationDeltaTime = config["simulation_delta_time"].value<double>();

        linearSpeedWeight = config["linear_speed_weight"].value<double>();
        angularSpeedWeight = config["angular_speed_weight"].value<double>();

        pathErrorWeight = config["path_error_weight"].value<double>();
        headingErrorWeight = config["heading_error_weight"].value<double>();
        goalWeight = config["goal_weight"].value<double>();
    }
};

}