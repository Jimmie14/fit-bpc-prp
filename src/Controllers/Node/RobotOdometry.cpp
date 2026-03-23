#include "RobotOdometry.hpp"
#include <cmath>

using namespace std;
using namespace rclcpp;

namespace Manhattan::Core
{

    // ---------------------------------------------------------------------------
    // Robot constants — keep in sync with MotorController.cpp
    // ---------------------------------------------------------------------------
    constexpr double WHEEL_RADIUS = 0.033;
    constexpr double WHEEL_BASE = 0.12;
    constexpr int32_t PULSES_PER_ROTATION = 550;

    constexpr auto ENCODERS_TOPIC = "/bpc_prp_robot/encoders";

    // ---------------------------------------------------------------------------

    RobotOdometry::RobotOdometry(const App& app) : BaseController(app),
        _kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION)
    {
        _posePub = _node->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
        _pathPub = _node->create_publisher<nav_msgs::msg::Path>("~/path", 10);

        _path.header.frame_id = "odom";

        Enable();
    }

    void RobotOdometry::Enable()
    {
        if (_encoderSub) return;

        _encoderSub = _node->create_subscription<std_msgs::msg::UInt32MultiArray>(
            ENCODERS_TOPIC, 10,
            [this](std_msgs::msg::UInt32MultiArray::SharedPtr msg) { onEncoders(std::move(msg)); }
        );

        RCLCPP_INFO(_node->get_logger(), "RobotOdometry enabled — encoder topic: %s", ENCODERS_TOPIC);
    }

    void RobotOdometry::Disable()
    {
        _encoderSub.reset();

        RCLCPP_INFO(_node->get_logger(), "RobotOdometry disabled");
    }

    void RobotOdometry::ApplyCorrection(const Pose2D& correctedPose)
    {
        _pose = correctedPose;
    }

    Kinematics RobotOdometry::GetKinematics() const {
        return _kinematics;
    }

    // ---------------------------------------------------------------------------
    // Private
    // ---------------------------------------------------------------------------

    void RobotOdometry::onEncoders(const std_msgs::msg::UInt32MultiArray::SharedPtr& msg)
    {
        if (msg->data.size() < 2)
        {
            RCLCPP_WARN_ONCE(_node->get_logger(), "Encoder message has fewer than 2 elements — ignoring.");
            return;
        }

        const int32_t rawLeft  = msg->data[0];
        const int32_t rawRight = msg->data[1];

        if (!_initialized)
        {
            _prevLeft = rawLeft;
            _prevRight = rawRight;
            _initialized = true;
            return;
        }
        // Delta ticks since last callback
        const int32_t dTicksLeft = rawLeft - _prevLeft;
        const int32_t dTicksRight = -(rawRight - _prevRight);

        _prevLeft = rawLeft;
        _prevRight = rawRight;

        // Convert to wheel linear distances (m)
        const double dLeft = _kinematics.ticksToMeters(dTicksLeft);
        const double dRight = _kinematics.ticksToMeters(dTicksRight);

        // Integrate pose
        _pose = _kinematics.integrate(_pose, dLeft, dRight);

        _linearVelocity  = (dLeft + dRight) * 0.5;
        _angularVelocity = (dRight - dLeft) / _kinematics.wheelBase();

        publishOdometry(_node->now());
    }

    void RobotOdometry::publishOdometry(const Time& stamp)
    {
        if ((stamp - _lastPublishTime).seconds() < 0.1) return;

        _lastPublishTime = stamp;

        const double halfTheta = _pose.theta * 0.5;
        const double qw = std::cos(halfTheta);
        const double qz = std::sin(halfTheta);

        // geometry_msgs/PoseStamped
        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header.stamp    = stamp;
        poseMsg.header.frame_id = "odom";
        poseMsg.pose.position.x = _pose.x;
        poseMsg.pose.position.y = _pose.y;
        poseMsg.pose.position.z = 0.0;
        poseMsg.pose.orientation.w = qw;
        poseMsg.pose.orientation.z = qz;
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;

        _posePub->publish(poseMsg);

        // nav_msgs/Path
        _path.header.stamp = stamp;
        _path.poses.push_back(poseMsg);
        if (_path.poses.size() > 1000)
            _path.poses.erase(_path.poses.begin());

        _pathPub->publish(_path);
    }

} // namespace Manhattan::Core

