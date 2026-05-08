#include "Components/MotorDriver.hpp"

#include "Messages/RobotMode.hpp"
#include "App.hpp"
#include "Components/OdometryEngine.hpp"
#include "Pid.hpp"

using namespace std;
using namespace rclcpp;
using namespace std_msgs;

namespace Manhattan::core {

constexpr auto MOTOR_SPEED_TOPIC = "/bpc_prp_robot/set_motor_speeds";
constexpr auto MOTOR_ENCODERS_TOPIC = "/bpc_prp_robot/encoders";

constexpr double ROTATIONS_PER_SECOND = 1.5;
constexpr double MAX_WHEEL_ANGULAR_SPEED = 2.0 * M_PI * ROTATIONS_PER_SECOND;
constexpr double ANGULAR_TO_SPEED = 1.0 / MAX_WHEEL_ANGULAR_SPEED;

struct MotorCharacteristics {
    double minSpeed;
    double maxSpeed;
};

struct MotorControllerConfig {
    MotorCharacteristics characteristics;

    double kp;
    double ki;
    double kd;
};

class MotorController {
public:
    explicit MotorController(const MotorControllerConfig& config)
        : _characteristics(config.characteristics)
        , _pid(config.kp, config.ki, config.kd)
    {
    }

    [[nodiscard]] double step(const double& desired, const double& actual, const double dt)
    {
        const auto error = desired - actual;

        const auto speed = desired + _pid.step(error, dt);

        return saturate(speed);
    }

    void reset()
    {
        _pid.reset();
    }
private:
    MotorCharacteristics _characteristics;
    Pid _pid;

    [[nodiscard]] double saturate(const double& value) const
    {
        if (std::abs(value) < _characteristics.minSpeed) return 0.0;

        return std::clamp(value, -_characteristics.maxSpeed, _characteristics.maxSpeed);
    }
};

MotorDriver::MotorDriver(const App& app)
    : RosDeviceDriver(app, "motor")
{
    _publisher = create_publisher<msg::UInt8MultiArray>(MOTOR_SPEED_TOPIC, 1);

    _msg = msg::UInt8MultiArray();
    _msg.data.push_back(127);
    _msg.data.push_back(127);

    const auto geometry = _app.getConfig<config::DifferentialDriveGeometry>("geometry");
    const auto kinematics = DifferentialDriveKinematics(geometry);

    _app.events->Subscribe<MotorCommandEvent>([this, kinematics](const MotorCommandEvent& command) {
        const auto [left, right] = kinematics.inverse(command.twist);

        SetForce(left, right);
    });

    _app.events->Subscribe<RobotModeChangeEvent>([this](const RobotModeChangeEvent& event) {
        _mode = event.newMode;
    });

    Enable();
}

void MotorDriver::OnEnable()
{
    _timer = create_wall_timer(100ms, [this] {
        if (_mode.motorOff) return;

        _publisher->publish(_msg);
    });
}

void MotorDriver::OnDisable()
{
    _timer.reset();
}

void MotorDriver::SetForce(const double leftAngular, const double rightAngular)
{
    auto left = leftAngular * ANGULAR_TO_SPEED;
    auto right = rightAngular * ANGULAR_TO_SPEED;

    left = clamp(left, -1.0, 1.0);
    right = clamp(right, -1.0, 1.0);

    _msg.data[0] = static_cast<uint8_t>((left * .5 + .5) * 255);
    _msg.data[1] = static_cast<uint8_t>((right * .5 + .5) * 255);
}
} // namespace Manhattan::Core