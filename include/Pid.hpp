#pragma once

class Pid {
public:
    Pid(const double kp, const double ki, const double kd)
        : _kp(kp), _ki(ki), _kd(kd), _prevError(0), _integral(0) {}

    double step(const double error, const double dt) {
        _integral += error * dt;

        const auto derivative = (error - _prevError) / dt;
        const auto output = _kp * error + _ki * _integral + _kd * derivative;

        _prevError = error;
        return output;
    }

    void reset() {
        _prevError = 0;
        _integral = 0;
    }

    void SetKp(const double kp)
    {
        _kp = kp;
    }

    void SetKi(const double ki)
    {
        _ki = ki;
    }

    void SetKd(const double kd)
    {
        _kd = kd;
    }

private:
    double _kp;
    double _ki;
    double _kd;
    double _prevError;
    double _integral;
};
