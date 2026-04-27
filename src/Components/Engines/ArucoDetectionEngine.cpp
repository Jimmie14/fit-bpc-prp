#include "ArucoDetectionEngine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

constexpr auto imageTopic = "/bpc_prp_robot/camera/compressed";
constexpr auto markerSize = 0.05;

namespace Manhattan::Core {
ArucoDetectionEngine::ArucoDetectionEngine(const App& app)
    : RosEngine(app, "aruco_detection")
{
}

void ArucoDetectionEngine::OnEnable()
{
    _imageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
        imageTopic, 1, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            this->OnImage(msg);
        });

    _posePublisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/poses", 1);
    _debugPublisher = this->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 1);
}

void ArucoDetectionEngine::OnDisable()
{
    _imageSubscription.reset();
}

void ArucoDetectionEngine::OnImage(const sensor_msgs::msg::Image::SharedPtr& msg)
{
    std::cout << "Image received" << std::endl;
}

}
