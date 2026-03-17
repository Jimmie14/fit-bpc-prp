//
// Created by guest on 3/17/26.
//

#include "Controllers/Node/LineController.hpp"

using namespace std;
using namespace rclcpp;

namespace Manhattan::Core
{
    LineController::LineController() : NodeController("LineController")
    {
        const auto subscriber = "/bpc_prp_robot/line_sensors";

        _subscriber = _node->create_subscription<std_msgs::msg::UInt16MultiArray>(
            subscriber, 1, std::bind(&LineController::OnLineSensorMsg, this, std::placeholders::_1));
    }


    float LineController::GetContinuousLinePose() const
    {
        return 0;
    }

    DiscreteLinePose LineController::GetDiscreteLinePose() const
    {
        return DiscreteLinePose::LineBoth;
    }

    void LineController::OnLineSensorMsg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        const auto length = msg->data.size();
        if (length != 2) return;

        auto estimation = LineEstimator::EstimateDiscrete(msg->data[0], msg->data[1]);
        if (estimation == _linePose) return;

        if (estimation == DiscreteLinePose::LineBoth)
            RCLCPP_INFO(_node->get_logger(), "Both sensors on line");
        else if (estimation == DiscreteLinePose::LineOnRight)
            RCLCPP_INFO(_node->get_logger(), "Right sensors on line");
        else if (estimation == DiscreteLinePose::LineOnLeft)
            RCLCPP_INFO(_node->get_logger(), "Left sensors on line");
        else
            RCLCPP_INFO(_node->get_logger(), "Nne sensors on line");

        _linePose = estimation;
    }
} // Manhattan