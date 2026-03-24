//
// Created by guest on 3/17/26.
//

#include "Controllers/Node/LineController.hpp"

#include "App.h"

using namespace std;
using namespace rclcpp;
using namespace std_msgs;

namespace Manhattan::Core
{
    constexpr auto subscriber = "/bpc_prp_robot/line_sensors";

    LineController::LineController(const App& app) : BaseController(app),
        _lineEstimator(0, 1000),
        _linePid(0.36f, 0.03f, 0.02f)
    {
        _motorController = _app.GetController<MotorController>();
    }

    Pid& LineController::GetPid()
    {
        return _linePid;
    }

    void LineController::SetMaxSpeed(const double speed)
    {
        _baseForce = speed;
    }

    void LineController::Enable()
    {
        if (_subscriber) return;

        _subscriber = _node->create_subscription<std_msgs::msg::UInt16MultiArray>(
            subscriber, 1, std::bind(&LineController::OnLineSensorMsg, this, std::placeholders::_1));

        _motorController = _app.GetController<MotorController>();


        // _lineSensorsPublisher = _node->create_publisher<msg::Float64MultiArray>("~/line_sensors", 10);
        // _linePosePublisher = _node->create_publisher<msg::Float64>("~/line_pose", 10);

        // _lineEstimator = LineEstimator(0, 1000);
        _linePid.reset();

        // _linePid.SetKp(0.36);
        // _linePid.SetKi(0.03);
        // _linePid.SetKd(0.02);

        _lastPidTime = _node->now();
        RCLCPP_INFO(_node->get_logger(), "Line controller enabled");
    }

    void LineController::Disable()
    {
        _subscriber.reset();

        // _lineSensorsPublisher.reset();
        // _linePosePublisher.reset();

        RCLCPP_INFO(_node->get_logger(), "Line controller disabled");
    }

    float LineController::GetContinuousLinePose() const
    {
        return 0;
    }

    DiscreteLinePose LineController::GetDiscreteLinePose() const
    {
        return DiscreteLinePose::LineBoth;
    }

    void LineController::OnLineSensorMsg(const msg::UInt16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 2) return;

        msg::Float64MultiArray sensors;
        sensors.data.push_back(_lineEstimator.NormalizeValue(msg->data[0], SensorLocation::Left));
        sensors.data.push_back(_lineEstimator.NormalizeValue(msg->data[1], SensorLocation::Right));
        // _lineSensorsPublisher->publish(sensors);

        const auto estimation = _lineEstimator.EstimateContinuousLinePose(msg->data[0], msg->data[1]);

        // publish current
        msg::Float64 linePose;
        linePose.data = estimation;
        // _linePosePublisher->publish(linePose);

        const auto now = _node->now();
        const auto dt = std::max(1e-3, (now - _lastPidTime).seconds());
        _lastPidTime = now;

        const auto error = 0.0 - estimation;
        const auto correction = std::clamp(_linePid.step(error, dt), -_maxCorrection, _maxCorrection);

        const auto leftForce = _baseForce - correction;
        const auto rightForce = _baseForce + correction;

        _motorController->SetForce(leftForce, rightForce);


        // auto estimation = _lineEstimator.EstimateDiscrete(msg->data[0], msg->data[1]);
        //
        // if (estimation == _linePose) return;
        //
        // if (estimation == DiscreteLinePose::LineBoth)
        //     _motorController->SetForce(1, 1);
        // else if (estimation == DiscreteLinePose::LineOnRight)
        //     _motorController->SetForce(1, .5);
        // else if (estimation == DiscreteLinePose::LineOnLeft)
        //     _motorController->SetForce(.5, 1);
        // else
        // {
        //     if (_linePose == DiscreteLinePose::LineOnRight)
        //         _motorController->SetForce(0.3, -.3);
        //     else if (_linePose == DiscreteLinePose::LineOnLeft)
        //         _motorController->SetForce(-.3, 0.3);
        //     else
        //         _motorController->SetForce(-1, 1);
        // }
        //
        // _linePose = estimation;
    }
} // Manhattan