#include "Components/MotorCalibrationUnit.hpp"

#include "App.hpp"

#include "Config/MotorConfig.hpp"
#include "Messages/Nav.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <vector>

using namespace std;
using namespace rclcpp;
using namespace std_msgs;

namespace Manhattan::core {

using namespace Manhattan::kinematics;
using namespace Manhattan::config;

constexpr int neutralPwm = 127;
constexpr int maxPwmOffset = 127;
constexpr auto updatePeriod = std::chrono::milliseconds(50);
constexpr int samplesPerStep = 5;

constexpr auto leftTopAngularSpeed = M_PI * 8;
constexpr auto rightTopAngularSpeed = M_PI * 8;

double median(std::vector<double> values)
{
    if (values.empty()) return 0.0;

    ranges::sort(values);
    const auto mid = values.size() / 2;

    if (values.size() % 2 == 0) {
        return (values[mid - 1] + values[mid]) * 0.5;
    }

    return values[mid];
}

double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

MotorCalibrationUnit::MotorCalibrationUnit(const App& app)
    : RosUnit(app, "motor_calibration")
    , _kinematics({})
{
    _app.config->watch<OdometryConfig>("odometry", [this](const OdometryConfig& config) {
        _odometryConfig = config;
    });

    _app.config->watch<MotorDriverConfig>("motors", [this](const MotorDriverConfig& config) {
        _motorConfig = config;
    });

    _app.config->watch<DifferentialDriveGeometry>("geometry", [this](const DifferentialDriveGeometry& geometry) {
        _kinematics = DifferentialDriveKinematics(geometry);
        _geometry = geometry;
    });

    _motorPublisher = create_publisher<msg::UInt8MultiArray>(_motorConfig.topic, 1);

    _motorMsg.data = {
        static_cast<uint8_t>(neutralPwm),
        static_cast<uint8_t>(neutralPwm)
    };
}

void MotorCalibrationUnit::OnEnable()
{
    _currentPwmOffset = 0;
    _samplesAtThisStep = 0;
    _sumAngularSpeed = 0.0;
    _initialized = false;
    _leftTable.clear();
    _rightTable.clear();
    _sweepEnabled = false;
    _phaseSamples.clear();
    _phaseStartTime = std::chrono::steady_clock::now();

    _encoderSubscriber = create_subscription<std_msgs::msg::UInt32MultiArray>(
        _odometryConfig.encodersTopic, 10, [this](const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
            onEncoders(msg);
        });

    _buttonsTopic = create_subscription<std_msgs::msg::UInt8>(
        "/bpc_prp_robot/buttons", 1, [this](const std_msgs::msg::UInt8 msg) {
            if (_phase != Phase::WaitForLift) return;

            _sweepEnabled = true;
            _phase = Phase::LeftForward;
            _currentPwmOffset = 0;
            _samplesAtThisStep = 0;
            _sumAngularSpeed = 0.0;
            _phaseSamples.clear();
            _phaseStartTime = std::chrono::steady_clock::now();

            RCLCPP_INFO(get_logger(), "Sweep calibration resumed");
        });

    _timer = create_wall_timer(updatePeriod, [this] {
        update();
    });

    RCLCPP_INFO(get_logger(), "Motor calibration started");
    RCLCPP_INFO(get_logger(), "Phase: EncoderLeft");
}

void MotorCalibrationUnit::OnDisable()
{
    setMotorPwm(0, 0);
    _timer.reset();
    _encoderSubscriber.reset();
    _buttonsTopic.reset();
}

void MotorCalibrationUnit::onEncoders(const std_msgs::msg::UInt32MultiArray::SharedPtr& msg)
{
    if (msg->data.size() < 2) return;

    _rawLeft = static_cast<int32_t>(msg->data[0]);
    _rawRight = static_cast<int32_t>(msg->data[1]);
}

void MotorCalibrationUnit::setMotorPwm(const int leftOffset, const int rightOffset)
{
    const auto left = std::clamp(neutralPwm + leftOffset, 0, 255);
    const auto right = std::clamp(neutralPwm + rightOffset, 0, 255);

    _motorMsg.data[0] = static_cast<uint8_t>(left);
    _motorMsg.data[1] = static_cast<uint8_t>(right);

    _motorPublisher->publish(_motorMsg);
}

void MotorCalibrationUnit::advancePhase()
{
    _currentPwmOffset = 0;
    _samplesAtThisStep = 0;
    _sumAngularSpeed = 0.0;
    _phaseSamples.clear();
    _phaseStartTime = std::chrono::steady_clock::now();

    switch (_phase) {
    case Phase::WaitForLift:
        break;

    case Phase::LeftForward:
        _phase = Phase::LeftReverse;
        RCLCPP_INFO(get_logger(), "Phase: LeftReverse");
        break;

    case Phase::LeftReverse:
        _phase = Phase::RightForward;
        RCLCPP_INFO(get_logger(), "Phase: RightForward");
        break;

    case Phase::RightForward:
        _phase = Phase::RightReverse;
        RCLCPP_INFO(get_logger(), "Phase: RightReverse");
        break;

    case Phase::RightReverse: {
        _phase = Phase::Done;
        setMotorPwm(0, 0);

        std::cout << "--- left ---" << std::endl;
        std::cout << tableToToml(_leftTable) << std::endl;

        std::cout << "--- right ---" << std::endl;
        std::cout << tableToToml(_rightTable) << std::endl;
        Disable();
        break;
    }
    case Phase::Done:
        break;
    }
}

std::string MotorCalibrationUnit::tableToToml(std::vector<CalibrationPoint>& table) const
{
    ranges::sort(table, [](const CalibrationPoint& a, const CalibrationPoint& b) {
        return a.pwm < b.pwm;
    });

    std::ostringstream result;

    result << "samples = [\n";

    for (const auto& [pwm, angularSpeed] : _leftTable) {
        result << "  [" << std::to_string(pwm) << ", ";
        result << angularSpeed << "],\n";
    }

    result << "]\n";

    return result.str();
}

void MotorCalibrationUnit::update()
{
    if (_phase == Phase::Done) return;

    const auto now = std::chrono::steady_clock::now();

    if (!_initialized) {
        _prevLeft = _rawLeft;
        _prevRight = _rawRight;
        _lastTime = now;
        _initialized = true;
        return;
    }

    const duration<double> delta = now - _lastTime;
    const auto dt = delta.count();
    if (dt <= 1e-6) return;

    _lastTime = now;

    const int32_t dTicksLeft = _rawLeft - _prevLeft;
    const int32_t dTicksRight = -(_rawRight - _prevRight);

    _prevLeft = _rawLeft;
    _prevRight = _rawRight;

    const auto leftAngular = _geometry.left.ticksToMeters(dTicksLeft) / _geometry.left.radius / dt;
    const auto rightAngular = _geometry.right.ticksToMeters(dTicksRight) / _geometry.right.radius / dt;

    if (_phase == Phase::WaitForLift) {
        setMotorPwm(0, 0);
        return;
    }

    double measuredAngular = 0.0;
    int signedPwm = 0;

    switch (_phase) {
    case Phase::LeftForward:
        signedPwm = _currentPwmOffset;
        setMotorPwm(signedPwm, 0);
        measuredAngular = leftAngular;
        break;

    case Phase::LeftReverse:
        signedPwm = -_currentPwmOffset;
        setMotorPwm(signedPwm, 0);
        measuredAngular = leftAngular;
        break;

    case Phase::RightForward:
        signedPwm = _currentPwmOffset;
        setMotorPwm(0, signedPwm);
        measuredAngular = rightAngular;
        break;

    case Phase::RightReverse:
        signedPwm = -_currentPwmOffset;
        setMotorPwm(0, signedPwm);
        measuredAngular = rightAngular;
        break;

    case Phase::WaitForLift:
    case Phase::Done:
        return;
    }

    _sumAngularSpeed += measuredAngular;
    ++_samplesAtThisStep;

    if (_samplesAtThisStep < samplesPerStep) return;

    const auto avgAngular = _sumAngularSpeed / static_cast<double>(_samplesAtThisStep);

    const uint8_t pwm = neutralPwm + signedPwm;

    if (_phase == Phase::LeftForward || _phase == Phase::LeftReverse) {
        _leftTable.push_back({ pwm, -avgAngular });
    } else {
        _rightTable.push_back({ pwm, avgAngular });
    }

    _samplesAtThisStep = 0;
    _sumAngularSpeed = 0.0;
    ++_currentPwmOffset;

    if (_currentPwmOffset > maxPwmOffset) {
        setMotorPwm(0, 0);
        advancePhase();
    }
}

} // namespace Manhattan::core