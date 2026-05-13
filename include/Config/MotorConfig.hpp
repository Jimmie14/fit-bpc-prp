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

struct MotorCalibrationConfig : Configurable {
    struct CalibrationPoint {
        uint8_t pwm;
        double angular;
    };

    std::vector<CalibrationPoint> table;

    void configure(const Config& config) override
    {
        const auto& arr = config;

        table.clear();
        table.reserve(arr.size());

        for (const auto& v : arr) {
            auto point = CalibrationPoint(v.at(0).value<uint8_t>(), v.at(1).value<double>());
            table.push_back(point);
        }

        ranges::sort(table, [](const auto& a, const auto& b) {
            return a.pwm < b.pwm;
        });
    }
};

struct MotorControllerConfig : Configurable {
    double kp;
    double ki;
    double kd;

    MotorCharacteristics characteristics;
    MotorCalibrationConfig calibration;

    void configure(const Config& config) override
    {
        kp = config["kp"].value<double>();
        ki = config["ki"].value<double>();
        kd = config["kd"].value<double>();

        characteristics.configure(config["characteristics"]);
        calibration.configure(config["samples"]);
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