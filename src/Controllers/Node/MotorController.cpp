#include "MotorController.hpp"

using namespace std;
using namespace rclcpp;
using namespace std_msgs;

namespace Manhattan::Core
{
    constexpr auto MOTOR_SPEED_TOPIC = "/bpc_prp_robot/set_motor_speeds";
    constexpr auto MOTOR_ENCODERS_TOPIC = "/bpc_prp_robot/encoders";

    constexpr double ROTATIONS_PER_SECOND = 1.5;
    constexpr double MAX_WHEEL_ANGULAR_SPEED = 2.0 * M_PI * ROTATIONS_PER_SECOND;
    constexpr double ANGULAR_TO_SPEED = 1.0 / MAX_WHEEL_ANGULAR_SPEED;

    MotorController::MotorController(const App& app) : BaseController(app)
    {
        _publisher = _node->create_publisher<msg::UInt8MultiArray>(MOTOR_SPEED_TOPIC, 1);

        _msg = msg::UInt8MultiArray();
        _msg.data.push_back(127);
        _msg.data.push_back(127);

        Enable();
    }

    void MotorController::OnEnable()
    {
        if (_subscriber) return;

        _subscriber = _node->create_subscription<msg::UInt32MultiArray>(
            MOTOR_ENCODERS_TOPIC, 1, std::bind(&MotorController::SubscriberCallback, this, std::placeholders::_1));

        _timer = _node->create_wall_timer(
            100ms,
            [this] { _publisher->publish(_msg); }
        );

        RCLCPP_INFO(_node->get_logger(), "Motor controller enabled");
    }

    void MotorController::OnDisable()
    {
        _subscriber.reset();
        _timer.reset();

        RCLCPP_INFO(_node->get_logger(), "Motor controller disabled");
    }

    void MotorController::SetForce(double leftAngular, double rightAngular)
    {
        auto left = leftAngular * ANGULAR_TO_SPEED;
        auto right = rightAngular * ANGULAR_TO_SPEED;

        left = clamp(left, -1.0, 1.0);
        right = clamp(right, -1.0, 1.0);

        _msg.data[0] = static_cast<uint8_t>((left * .5 + .5) * 255);
        _msg.data[1] = static_cast<uint8_t>((right * .5 + .5) * 255);
    }

    void MotorController::SubscriberCallback(const msg::UInt32MultiArray::SharedPtr msg) const
    {
        return;
        const auto length = msg->data.size();

        if (length >= 1)
            RCLCPP_INFO(_node->get_logger(), "Left wheel turned: %u", msg->data[0]);

        if (length >= 2)
            RCLCPP_INFO(_node->get_logger(), "Right wheel turned: %u", msg->data[1]);
    }
}