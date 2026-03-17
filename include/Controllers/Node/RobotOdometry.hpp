#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#include "Kinematics.hpp"
#include "NodeController.hpp"

namespace Manhattan::Core
{
    class RobotOdometry : public NodeController
    {
    public:
        RobotOdometry();

        void ApplyCorrection(const Pose2D& correctedPose);

        [[nodiscard]] Kinematics GetKinematics() const;

    private:
        void onEncoders(std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        void publishOdometry(const rclcpp::Time& stamp) const;

        // -----------------------------------------------------------------------
        Kinematics _kinematics;

        Pose2D _pose = { };
        double _linearVelocity = 0.0;
        double _angularVelocity = 0.0;

        // Previous cumulative encoder counts
        int32_t _prevLeft = 0;
        int32_t _prevRight = 0;
        bool _initialized = false;

        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr _encoderSub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           _odomPub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   _posePub;
    };
}
