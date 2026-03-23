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
    _encoderSub = _node->create_subscription<std_msgs::msg::UInt32MultiArray>(
        ENCODERS_TOPIC, 10,
        [this](std_msgs::msg::UInt32MultiArray::SharedPtr msg) { onEncoders(std::move(msg)); }
    );

    _odomPub = _node->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    _posePub = _node->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);

    RCLCPP_INFO(_node->get_logger(), "RobotOdometry started — encoder topic: %s", ENCODERS_TOPIC);
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

void RobotOdometry::onEncoders(std_msgs::msg::UInt32MultiArray::SharedPtr msg)
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

void RobotOdometry::publishOdometry(const Time& stamp) const {
    const double halfTheta = _pose.theta * 0.5;
    const double qw = std::cos(halfTheta);
    const double qz = std::sin(halfTheta);

    // --- nav_msgs/Odometry --------------------------------------------------
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = _pose.x;
    odom.pose.pose.position.y = _pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;

    odom.twist.twist.linear.x = _linearVelocity;
    odom.twist.twist.angular.z = _angularVelocity;

    _odomPub->publish(odom);

    // --- geometry_msgs/PoseStamped  (lightweight, easy to plot/debug) -------
    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header = odom.header;
    poseMsg.pose = odom.pose.pose;

    _posePub->publish(poseMsg);
}

} // namespace Manhattan::Core

