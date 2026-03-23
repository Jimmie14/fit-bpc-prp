#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include "BaseController.h"
#include "LineEstimator.hpp"
#include "MotorController.hpp"

namespace Manhattan::Core
{
    class LineController final : public BaseController {
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr _subscriber;
        DiscreteLinePose _linePose = DiscreteLinePose::LineNone;

        std::shared_ptr<MotorController> _motorController;

        LineEstimator _lineEstimator;
    public:
        explicit LineController(const App& app);

        // Relative pose to line [m]
        [[nodiscard]] float GetContinuousLinePose() const;

        [[nodiscard]] DiscreteLinePose GetDiscreteLinePose() const;

        void Enable() override;

        void Disable() override;

    private:
        void OnLineSensorMsg(std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    };
} // Manhattan::Core