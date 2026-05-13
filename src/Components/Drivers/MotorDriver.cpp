#include "Components/MotorDriver.hpp"

#include "Messages/RobotMode.hpp"
#include "App.hpp"
#include "Components/OdometryEngine.hpp"
#include "Pid.hpp"

using namespace std;
using namespace rclcpp;
using namespace std_msgs;

namespace Manhattan::core {

MotorController::MotorController(const config::MotorControllerConfig& config)
    : _characteristics(config.characteristics)
    , _pid(config.kp, config.ki, config.kd)
    , _calibrationSolver(config.calibration)
{

}

double MotorController::step(const double desired, const double dt)
{
    const auto error = desired - _current;
    const auto speed = desired + _pid.step(error, dt);

    _current = speed;
    // _current = saturate(speed);

    return _current;
}

uint8_t MotorController::angularToPwm(const double angular) const
{
    return _calibrationSolver.angularToPwm(angular);
}

void MotorController::reset()
{
    _current = 0.0;
    _pid.reset();
}

double MotorController::saturate(const double& value) const
{
    if (std::abs(value) < _characteristics.minSpeed) return 0.0;

    return std::clamp(value, -_characteristics.maxSpeed, _characteristics.maxSpeed);
}

MotorDriver::MotorDriver(const App& app)
    : RosDeviceDriver(app, "motor")
    , _left({ })
    , _right({ })
{
    _app.config->watch<MotorDriverConfig>("motors", [this](const MotorDriverConfig& config) {
        _config = config;

        _left = MotorController(_config.left);
        _right = MotorController(_config.right);
    });

    _app.config->watch<DifferentialDriveGeometry>("geometry", [this](const DifferentialDriveGeometry& geometry) {
        _kinematics = DifferentialDriveKinematics(geometry);
    });

    _publisher = create_publisher<msg::UInt8MultiArray>(_config.topic, 1);

    _msg = msg::UInt8MultiArray();
    _msg.data.push_back(127);
    _msg.data.push_back(127);

    _app.events->Subscribe<MotorCommandEvent>([this](const MotorCommandEvent& command) {
        const auto [left, right] = _kinematics.inverse(command.twist);

        SetForce(left, right);
    });

    _app.events->Subscribe<RobotModeChangeEvent>([this](const RobotModeChangeEvent& event) {
        _mode = event.newMode;
    });

    Enable();
}

void MotorDriver::OnEnable()
{
    _timer = create_wall_timer(duration<double>(_config.deltaTime), [this] {
        Publish();
    });
}

void MotorDriver::OnDisable()
{
    _timer.reset();
}

void MotorDriver::SetForce(const double leftAngular, const double rightAngular)
{
    if (_mode.motorOff) {
        _desired = { 0.0, 0.0 };
        return;
    }

    _desired = { leftAngular, rightAngular };
}

void MotorDriver::Publish()
{
    auto left = _left.step(_desired.left, _config.deltaTime);
    auto right = _right.step(_desired.right, _config.deltaTime);

    const auto leftPwm = _left.angularToPwm(-left);
    const auto rightPwm = _right.angularToPwm(-right);

    _msg.data[0] = leftPwm;
    _msg.data[1] = rightPwm;

    _publisher->publish(_msg);
}

} // namespace Manhattan::Core