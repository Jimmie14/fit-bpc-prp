#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "BaseController.h"
#include "LineEstimator.hpp"
#include "MotorController.hpp"
#include "Pid.hpp"

namespace Manhattan::Core
{
    class LineController: public BaseController {
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr _subscriber;
        DiscreteLinePose _linePose = DiscreteLinePose::LineNone;

        std::shared_ptr<MotorController> _motorController;

        LineEstimator _lineEstimator;

        Pid _linePid;

        double _baseForce = 0.75f;
        double _maxCorrection = 0.45f;
        double _lastContinuousError = 0.0f;
        rclcpp::Time _lastPidTime;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _linePosePublisher;




    public:
        explicit LineController(const App& app);

        // Relative pose to line [m]
        [[nodiscard]] float GetContinuousLinePose() const;

        [[nodiscard]] DiscreteLinePose GetDiscreteLinePose() const;

    private:
        void OnLineSensorMsg(std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    };
} // Manhattan::Core