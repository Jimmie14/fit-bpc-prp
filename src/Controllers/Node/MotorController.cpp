#include "Controllers/Node/MotorController.h"

using namespace std;
using namespace rclcpp;
using namespace std_msgs;

namespace Manhattan::Core
{
    MotorController::MotorController() : NodeController("MotorController")
    {
        const auto publisher = "/bpc_prp_robot/set_motor_speeds";
        const auto subscriber = "/bpc_prp_robot/encoders";

        _publisher = _node->create_publisher<msg::UInt8MultiArray>(publisher, 1);
        _subscriber = _node->create_subscription<msg::UInt32MultiArray>(
            subscriber, 1, std::bind(&MotorController::SubscriberCallback, this, std::placeholders::_1));

        _msg = msg::UInt8MultiArray();
        _msg.data.push_back(127);
        _msg.data.push_back(127);

        _timer = _node->create_wall_timer(
            100ms,
            [this] { Update(); }
        );
    }

    void MotorController::SetForce(double left, double right)
    {
        left = clamp(left, -1.0, 1.0);
        right = clamp(right, -1.0, 1.0);

        const auto leftVal = static_cast<uint8_t>((left * .5 + .5) * 255);
        const auto rightVal = static_cast<uint8_t>((right * .5 + .5) * 255);

        _msg.data[0] = leftVal;
        _msg.data[1] = rightVal;
    }

    void MotorController::Update() const
    {
        _publisher->publish(_msg);
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