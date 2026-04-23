#include "MotorDriver.hpp"

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

    Enable();
}

void MotorDriver::OnEnable()
{
    _timer = create_wall_timer(100ms, [this] {
        _publisher->publish(_msg);
    });

    RCLCPP_INFO(get_logger(), "Motor controller enabled");
}

void MotorDriver::OnDisable()
{
    _timer.reset();

    RCLCPP_INFO(get_logger(), "Motor controller disabled");
}

void MotorDriver::SetForce(double leftAngular, double rightAngular)
{
    auto left = leftAngular * ANGULAR_TO_SPEED;
    auto right = rightAngular * ANGULAR_TO_SPEED;

    left = clamp(left, -1.0, 1.0);
    right = clamp(right, -1.0, 1.0);

    _msg.data[0] = static_cast<uint8_t>((left * .5 + .5) * 255);
    _msg.data[1] = static_cast<uint8_t>((right * .5 + .5) * 255);
}
} // namespace Manhattan::Core