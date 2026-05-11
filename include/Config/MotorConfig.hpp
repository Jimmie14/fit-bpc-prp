#pragma once

#include "Configurable.hpp"

namespace Manhattan::config {
struct MotorCharacteristics : Configurable {
    double minSpeed;
    double maxSpeed;

    void configure(const Config& config) override
    {
        minSpeed = config["min_speed"].value<double>();
        maxSpeed = config["max_speed"].value<double>();
    }
};

struct MotorControllerConfig : Configurable {
    double kp;
    double ki;
    double kd;

    MotorCharacteristics characteristics;

    void configure(const Config& config) override
    {
        kp = config["kp"].value<double>();
        ki = config["ki"].value<double>();
        kd = config["kd"].value<double>();

        characteristics.configure(config["characteristics"]);
    }
};


struct MotorDriverConfig : Configurable {
    std::string topic;
    double deltaTime;

    MotorControllerConfig left;
    MotorControllerConfig right;


    void configure(const Config& config) override
    {
        topic = config["topic"].value<std::string>();
        deltaTime = config["delta_time"].value<double>();

        left.configure(config["left"]);
        right.configure(config["right"]);
    }
};



}