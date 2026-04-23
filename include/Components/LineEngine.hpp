#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include "LineEstimator.hpp"
#include "MotorDriver.hpp"
#include "Pid.hpp"
#include "RosEngine.hpp"

namespace Manhattan::Core {
class LineEngine final : public RosEngine {
    Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr _subscriber;
    DiscreteLinePose _linePose = DiscreteLinePose::LineNone;

    std::shared_ptr<MotorDriver> _motorController;

    LineEstimator _lineEstimator;

    Pid _linePid;

    double _baseForce = 0.75f;
    double _maxCorrection = 0.45f;
    double _lastContinuousError = 0.0f;
    Time _lastPidTime;

    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _lineSensorsPublisher;
    Publisher<std_msgs::msg::Float64>::SharedPtr _linePosePublisher;

public:
    explicit LineEngine(const App& app);

    // Relative pose to line [m]
    [[nodiscard]] float GetContinuousLinePose() const;

    [[nodiscard]] DiscreteLinePose GetDiscreteLinePose() const;

    [[nodiscard]] Pid& GetPid();

    void SetMaxSpeed(double speed);

    void OnEnable() override;

    void OnDisable() override;

private:
    void OnLineSensorMsg(std_msgs::msg::UInt16MultiArray::SharedPtr msg);
};
} // namespace Manhattan::Core