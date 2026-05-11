#pragma once

#include "Messages/RobotMode.hpp"
#include "Pid.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "Common/RosDeviceDriver.hpp"
#include "Config/MotorConfig.hpp"
#include "Kinematics/Kinematics.hpp"

namespace Manhattan::core {

using namespace Manhattan::messages;

class MotorController {
public:
    explicit MotorController(const config::MotorControllerConfig& config);

    [[nodiscard]] double step(double desired, double dt);

    void reset();
private:
    config::MotorCharacteristics _characteristics;

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
