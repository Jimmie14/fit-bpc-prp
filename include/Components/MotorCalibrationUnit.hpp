#pragma once

#include "Common/RosUnit.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "Kinematics/Kinematics.hpp"
#include "Config/OdometryConfig.hpp"
#include "Config/MotorConfig.hpp"

#include <std_msgs/msg/u_int8.hpp>

namespace Manhattan::core {

class MotorCalibrationUnit : public RosUnit {
public:
    explicit MotorCalibrationUnit(const App& app);

protected:
    void OnEnable() override;
    void OnDisable() override;

private:
    struct CalibrationPoint {
        uint8_t pwm;
        double angularSpeed;
    };

    enum class Phase {
        // EncoderLeft,
        // EncoderRight,
        WaitForLift,
        LeftForward,
        LeftReverse,
        RightForward,
        RightReverse,
        Done
    };

    std::string tableToToml(std::vector<CalibrationPoint>& table) const;

    void onEncoders(const std_msgs::msg::UInt32MultiArray::SharedPtr& msg);
    void update();
    void setMotorPwm(int leftOffset, int rightOffset);
    void advancePhase();

    TimerBase::SharedPtr _timer;
    Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr _encoderSubscriber;
    Subscription<std_msgs::msg::UInt8>::SharedPtr _buttonsTopic;
    Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _motorPublisher;

    std::chrono::steady_clock::time_point _lastTime;
    std::chrono::steady_clock::time_point _phaseStartTime;

    config::OdometryConfig _odometryConfig;
    config::MotorDriverConfig _motorConfig;
    config::DifferentialDriveGeometry _geometry;
    kinematics::DifferentialDriveKinematics _kinematics;

    int32_t _rawLeft = 0;
    int32_t _rawRight = 0;
    int32_t _prevLeft = 0;
    int32_t _prevRight = 0;

    bool _initialized = false;
    bool _sweepEnabled = false;

    Phase _phase = Phase::WaitForLift;

    int _currentPwmOffset = 0;
    int _samplesAtThisStep = 0;
    double _sumAngularSpeed = 0.0;

    std::vector<double> _phaseSamples;
    std::vector<CalibrationPoint> _leftTable;
    std::vector<CalibrationPoint> _rightTable;

    std_msgs::msg::UInt8MultiArray _motorMsg;
};
}