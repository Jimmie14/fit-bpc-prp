#pragma once

#include "Configurable.hpp"

namespace Manhattan::config {
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

    double progressRewardWeight;
    double pathErrorCostWeight;
    double headingErrorCostWeight;

    void configure(const Config& config) override
    {
        deltaTime = config["delta_time"].value<double>();
        lookaheadDistance = config["lookahead_distance"].value<double>();

        linearAcceleration = config["linear_acceleration"].value<double>();
        angularAcceleration = config["angular_acceleration"].value<double>();

        maxLinearSpeed = config["max_linear_speed"].value<double>();
        maxAngularSpeed = config["max_angular_speed"].value<double>();

        linearVelocitySamples = config["linear_velocity_samples"].value<int>();
        angularVelocitySamples = config["angular_velocity_samples"].value<int>();

        simulationSteps = config["simulation_steps"].value<int>();
        simulationDeltaTime = config["simulation_delta_time"].value<double>();

        progressRewardWeight = config["progress_reward_weight"].value<double>();
        pathErrorCostWeight = config["path_error_cost_weight"].value<double>();
        headingErrorCostWeight = config["heading_error_cost_weight"].value<double>();
    }
};

}