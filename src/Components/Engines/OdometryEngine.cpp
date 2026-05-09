#include "Components/OdometryEngine.hpp"

#include "App.hpp"

using namespace std;
using namespace rclcpp;

namespace Manhattan::core {


OdometryEngine::OdometryEngine(const App& app)
    : RosEngine(app, "odometry")
    , _kinematics({ })
    , _odometry({ })
{
    _app.config->watch<OdometryConfig>("odometry", [this](const OdometryConfig& config) {
        _config = config;
    });

    _app.config->watch<DifferentialDriveGeometry>("geometry", [this](const DifferentialDriveGeometry& geometry) {
        _kinematics = DifferentialDriveKinematics(geometry);
        _odometry = DifferentialDriveOdometry(geometry);
    });

    _odomPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    _posePublisher = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
}

void OdometryEngine::OnEnable()
{
    _encoderSubscriber = create_subscription<std_msgs::msg::UInt32MultiArray>(
        _config.encodersTopic, 10, [this](const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
            OnEncoders(msg);
        });
}

void OdometryEngine::OnDisable()
{
    _encoderSubscriber.reset();
}

void OdometryEngine::ApplyCorrection(const Pose& correctedPose)
{
    _pose = correctedPose;
}

void OdometryEngine::OnEncoders(const std_msgs::msg::UInt32MultiArray::SharedPtr& msg)
{
    if (msg->data.size() < 2) return;

    const auto now = std::chrono::steady_clock::now();
    const duration<double> delta = now - _lastTime;
    const auto deltaTime = delta.count();

    _lastTime = now;

    const int32_t rawLeft = msg->data[0];
    const int32_t rawRight = msg->data[1];

    if (!_initialized) {
        _prevLeft = rawLeft;
        _prevRight = rawRight;
        _initialized = true;
        return;
    }
    const int32_t dTicksLeft = rawLeft - _prevLeft;
    const int32_t dTicksRight = -(rawRight - _prevRight);

    _prevLeft = rawLeft;
    _prevRight = rawRight;


    const auto linearStep = _kinematics.ticksToLinear(dTicksLeft, dTicksRight);
    _pose = _odometry.integrate(_pose, linearStep);


    auto twist = _kinematics.forward(_kinematics.linearToAngular(linearStep));

    twist.linear /= deltaTime;
    twist.angular /= deltaTime;

    const auto alpha = 1 - exp(-deltaTime / 0.1);

    _twist.linear = _twist.linear * (1.0 - alpha) + twist.linear * alpha;
    _twist.angular = _twist.angular * (1.0 - alpha) + twist.angular * alpha;

    publishOdometry();
}

void OdometryEngine::publishOdometry() const
{
    // nav_msgs/odom
    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header.stamp = now();
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "base_link";

    // position
    odomMsg.pose.pose = _pose.ToRosPoseMessage();

    // velocity
    odomMsg.twist.twist.linear.x = _twist.linear;
    odomMsg.twist.twist.linear.y = 0.0;
    odomMsg.twist.twist.linear.z = 0.0;
    odomMsg.twist.twist.angular.x = 0.0;
    odomMsg.twist.twist.angular.y = 0.0;
    odomMsg.twist.twist.angular.z = _twist.angular;

    // Pose covariance (x, y, z, roll, pitch, yaw)
    odomMsg.pose.covariance = { 0.05, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 999, 0, 0, 0,
        0, 0, 0, 999, 0, 0, 0, 0, 0, 0, 999, 0, 0, 0, 0, 0, 0, 0.2 };

    // Twist covariance (vx, vy, vz, vroll, vpitch, vyaw)
    odomMsg.twist.covariance = { 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 999, 0, 0, 0,
        0, 0, 0, 999, 0, 0, 0, 0, 0, 0, 999, 0, 0, 0, 0, 0, 0, 0.2 };

    _odomPublisher->publish(odomMsg);

    // geometry_msgs/PoseStamped
    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header.stamp = now();
    poseMsg.header.frame_id = "odom";
    poseMsg.pose = odomMsg.pose.pose;

    _posePublisher->publish(poseMsg);
}

} // namespace Manhattan::Core
