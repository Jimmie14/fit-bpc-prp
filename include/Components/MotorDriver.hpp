#pragma once

#include "Messages/RobotMode.hpp"
#include "Pid.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "Common/RosDeviceDriver.hpp"
#include "Config/MotorConfig.hpp"
#include "Kinematics/Kinematics.hpp"
#include <ranges>

namespace Manhattan::core {

using namespace Manhattan::messages;

class MotorCalibrationSolver {
public:
    explicit MotorCalibrationSolver(const config::MotorCalibrationConfig& config)
        : _table(config.table)
    {}

    [[nodiscard]] uint8_t angularToPwm(const double targetAngular) const
    {
        if (_table.empty()) return 127;

        if (abs(targetAngular) < 0.001) return 127;

        uint8_t best = 127;
        double bestError = std::numeric_limits<double>::max();

        if (targetAngular < 0.0) {
            for (const auto& [pwm, angular] : _table) {
                if (pwm < 129) continue;

                const auto error = std::abs(angular - targetAngular);
                if (error < bestError) {
                    bestError = error;
                    best = pwm;
                }
            }
        } else {
            for (const auto& [pwm, angular] : _table) {
                if (pwm > 125) continue;

                const auto error = std::abs(angular - targetAngular);
                if (error < bestError) {
                    bestError = error;
                    best = pwm;
                }
            }
        }

        return best;
    }

private:
    std::vector<config::MotorCalibrationConfig::CalibrationPoint> _table;
};


class MotorController {
public:
    explicit MotorController(const config::MotorControllerConfig& config);

    [[nodiscard]] double step(double desired, double dt);

    [[nodiscard]] uint8_t angularToPwm(double angular) const;

    void reset();
private:
    config::MotorCharacteristics _characteristics;
    MotorCalibrationSolver _calibrationSolver;

    double _current = 0.0;
    Pid _pid;

    [[nodiscard]] double saturate(const double& value) const;
};

class MotorDriver final : public RosDeviceDriver {
public:
    explicit MotorDriver(const App& app);

    void SetForce(double leftAngular, double rightAngular);

protected:
    void OnEnable() override;

    void OnDisable() override;
private:
    Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _publisher;

    TimerBase::SharedPtr _timer;
    std_msgs::msg::UInt8MultiArray _msg;
    RobotMode _mode = { };

    kinematics::WheelAngularVelocity _desired = { };

    config::MotorDriverConfig _config;
    kinematics::DifferentialDriveKinematics _kinematics = kinematics::DifferentialDriveKinematics({ });

    MotorController _left;
    MotorController _right;

    void Publish();
};
} // namespace Manhattan::Core
