#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#include "BaseController.h"
#include "Kinematics.hpp"

namespace Manhattan::Core
{
    class RobotOdometry : public BaseController
    {
    public:
        explicit RobotOdometry(const App& app);

        void ApplyCorrection(const Pose2D& correctedPose);

        [[nodiscard]] Kinematics GetKinematics() const;

    private:
        void onEncoders(std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        void publishOdometry(const rclcpp::Time& stamp);

        // -----------------------------------------------------------------------
        Kinematics _kinematics;

        Pose2D _pose = { };
        double _linearVelocity = 0.0;
        double _angularVelocity = 0.0;

        // Previous cumulative encoder counts
        int32_t _prevLeft = 0;
        int32_t _prevRight = 0;
        bool _initialized = false;

        rclcpp::Time _lastPublishTime{0, 0, RCL_ROS_TIME};

        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr _encoderSub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _posePub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pathPub;

        nav_msgs::msg::Path _path;
    };
}
