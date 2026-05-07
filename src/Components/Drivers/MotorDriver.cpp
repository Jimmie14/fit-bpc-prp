#include "MotorDriver.hpp"

#include "App.hpp"
#include "OdometryEngine.hpp"
#include "RobotMode.hpp"

using namespace std;
using namespace rclcpp;
using namespace std_msgs;

namespace Manhattan::Core {
constexpr auto MOTOR_SPEED_TOPIC = "/bpc_prp_robot/set_motor_speeds";
constexpr auto MOTOR_ENCODERS_TOPIC = "/bpc_prp_robot/encoders";

constexpr double ROTATIONS_PER_SECOND = 1.5;
constexpr double MAX_WHEEL_ANGULAR_SPEED = 2.0 * M_PI * ROTATIONS_PER_SECOND;
constexpr double ANGULAR_TO_SPEED = 1.0 / MAX_WHEEL_ANGULAR_SPEED;

MotorDriver::MotorDriver(const App& app)
    : RosDeviceDriver(app, "motor")
{
    _publisher = create_publisher<msg::UInt8MultiArray>(MOTOR_SPEED_TOPIC, 1);

    _msg = msg::UInt8MultiArray();
    _msg.data.push_back(127);
    _msg.data.push_back(127);

    auto kinematics = app.GetComponent<OdometryEngine>()->GetKinematics();

    _app.Events->Subscribe<MotorCommand>([this, kinematics](const MotorCommand& command) {
        const auto [left, right] = kinematics.inverse(RobotSpeed { command.linear, command.angular });

        SetForce(left, right);
    });

    _app.Events->Subscribe<RobotModeChangeEvent>([this](const RobotModeChangeEvent& event) {
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

    if (_mode.reverse) {
        std::swap(left, right);

        left = -left;
        right = -right;
    }

    left = clamp(left, -1.0, 1.0);
    right = clamp(right, -1.0, 1.0);

    _msg.data[0] = static_cast<uint8_t>((left * .5 + .5) * 255);
    _msg.data[1] = static_cast<uint8_t>((right * .5 + .5) * 255);
}
} // namespace Manhattan::Core