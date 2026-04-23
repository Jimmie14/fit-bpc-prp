#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#include "Kinematics.hpp"
#include "RosComponent.hpp"
#include "RosEngine.hpp"

namespace Manhattan::Core {
class OdometryEngine final : public RosEngine {
public:
    explicit OdometryEngine(const App& app);

    void ApplyCorrection(const Pose& correctedPose);

    [[nodiscard]] Kinematics GetKinematics() const;

    void OnEnable() override;

    void OnDisable() override;

private:
    void OnEncoders(const std_msgs::msg::UInt32MultiArray::SharedPtr& msg);

    void publishOdometry(const Time& stamp);

    // -----------------------------------------------------------------------
    Kinematics _kinematics;

    Pose _pose = {};
    double _linearVelocity = 0.0;
    double _angularVelocity = 0.0;

    // Previous cumulative encoder counts
    int32_t _prevLeft = 0;
    int32_t _prevRight = 0;
    bool _initialized = false;

    Time _lastPublishTime { 0, 0, RCL_ROS_TIME };

    Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr _encoderSub;
    Publisher<nav_msgs::msg::Odometry>::SharedPtr _odomPub;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _posePub;
};
} // namespace Manhattan::Core
