#pragma once

class Pid {
public:
    Pid(const float kp, const float ki, const float kd)
        : _kp(kp), _ki(ki), _kd(kd), _prevError(0), _integral(0) {}

    float step(const float error, const float dt) {
        _integral += error * dt;

        const float derivative = (error - _prevError) / dt;
        const float output = _kp * error + _ki * _integral + _kd * derivative;

        _prevError = error;
        return output;
    }

    void reset() {
        _prevError = 0;
        _integral = 0;
    }

private:
    float _kp;
    float _ki;
    float _kd;
    float _prevError;
    float _integral;
};
