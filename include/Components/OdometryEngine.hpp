#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#include "Common/RosComponent.hpp"
#include "Common/RosEngine.hpp"
#include "Config/OdometryConfig.hpp"
#include "Kinematics/Kinematics.hpp"
#include "Math/LinearPath.hpp"
#include "Math/Pose.hpp"

namespace Manhattan::core {

using namespace Manhattan::math;
using namespace Manhattan::kinematics;

class OdometryEngine final : public RosEngine {
public:
    explicit OdometryEngine(const App& app);

    void ApplyCorrection(const Pose& correctedPose);

protected:
    void OnEnable() override;

    void OnDisable() override;

private:
    config::OdometryConfig _config;

    void OnEncoders(const std_msgs::msg::UInt32MultiArray::SharedPtr& msg);

    void publishOdometry() const;

    DifferentialDriveKinematics _kinematics;
    DifferentialDriveOdometry _odometry;

    std::chrono::steady_clock::time_point _lastTime;

    Pose _pose = {};
    Twist _twist = Twist::zero();

    int32_t _prevLeft = 0;
    int32_t _prevRight = 0;
    bool _initialized = false;

    Time _lastPublishTime { 0, 0, RCL_ROS_TIME };

    Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr _encoderSubscriber;
    Publisher<nav_msgs::msg::Odometry>::SharedPtr _odomPublisher;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _posePublisher;
};
} // namespace Manhattan::Core
