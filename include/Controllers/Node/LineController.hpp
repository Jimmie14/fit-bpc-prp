#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include "LineEstimator.hpp"
#include "NodeController.hpp"

namespace Manhattan::Core
{
    class LineController: public NodeController {
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr _subscriber;
        DiscreteLinePose _linePose = DiscreteLinePose::LineNone;

    public:
        LineController();

        // Relative pose to line [m]
        [[nodiscard]] float GetContinuousLinePose() const;

        [[nodiscard]] DiscreteLinePose GetDiscreteLinePose() const;

    private:
        void OnLineSensorMsg(std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    };
} // Manhattan::Core