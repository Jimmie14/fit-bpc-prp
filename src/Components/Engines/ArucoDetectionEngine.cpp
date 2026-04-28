#include "ArucoDetectionEngine.hpp"

#include "../../../include/Math/Vec3.hpp"
#include "Viz/Marker.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

constexpr auto imageTopic = "/bpc_prp_robot/camera/compressed";
constexpr auto cameraInfoTopic = "/bpc_prp_robot/camera_info";
constexpr auto markerSize = 0.065;

constexpr auto cameraHeight = 0.165;
constexpr auto cameraPitchToFloor = M_PI * 0.25;

namespace Manhattan::Core {

ArucoDetectionEngine::ArucoDetectionEngine(const App& app)
    : RosEngine(app, "aruco_detection")
{
    _mappingEngine = app.GetComponent<MappingEngine>();

    _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    _parameters = cv::aruco::DetectorParameters::create();

    // height: 16.5cm, pi/4 rad
    _hasCameraInfo = true;
    _cameraMatrix = (cv::Mat_<double>(3, 3) << 1312.66874, 0.0, 308.222153,
        0.0, 1316.64822, 298.881634,
        0.0, 0.0, 1.0);

    _distanceCoefficients = (cv::Mat_<double>(1, 5) << -0.263544599,
        6.59905618,
        0.0197602951,
        0.0000215995344,
        -36.5553743);

    _app.Events->Subscribe<MappingEngineStateChangeEvent>([this](const MappingEngineStateChangeEvent& event) {
        this->OnMappingEngineStateChange(event);
    });
}

void ArucoDetectionEngine::OnEnable()
{
    _imageSubscription = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        imageTopic, 1, [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            this->OnImage(msg);
        });

    // _cameraInfoSubscription = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    //     cameraInfoTopic, 1, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    //         this->OnCameraInfo(msg);
    //     });

    _markerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("aruco/markers", 1);
    _debugPublisher = this->create_publisher<sensor_msgs::msg::Image>("aruco/debug_image", 1);
}

void ArucoDetectionEngine::OnDisable()
{
    _imageSubscription.reset();

    _markerPublisher.reset();
    _debugPublisher.reset();
}

void ArucoDetectionEngine::OnCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr& msg)
{
    if (_hasCameraInfo) return;
    _hasCameraInfo = true;

    _cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    _cameraMatrix.at<double>(0, 0) = msg->k[0]; // fx
    _cameraMatrix.at<double>(1, 1) = msg->k[4]; // fy
    _cameraMatrix.at<double>(0, 2) = msg->k[2]; // cx
    _cameraMatrix.at<double>(1, 2) = msg->k[5]; // cy
    _cameraMatrix.at<double>(2, 2) = 1.0;

    _distanceCoefficients = cv::Mat(msg->d).clone();

    _hasCameraInfo = true;
}

void ArucoDetectionEngine::OnImage(const sensor_msgs::msg::CompressedImage::SharedPtr& msg)
{
    if (!_hasCameraInfo) return;

    const auto frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    if (frame.empty())
        return;

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

    cv::aruco::detectMarkers(gray, _dictionary, corners, ids, _parameters);

    const auto stamp = this->now();

    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.stamp = stamp;
    clear_marker.header.frame_id = "map";
    clear_marker.ns = "aruco";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(clear_marker);

    const auto robotPose = _mappingEngine->CurrentPose();

    if (!ids.empty()) {
        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::estimatePoseSingleMarkers(
            corners,
            markerSize,
            _cameraMatrix,
            _distanceCoefficients,
            rvecs,
            tvecs);

        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        for (size_t i = 0; i < ids.size(); i++) {
            const auto cameraRight = tvecs[i][0];
            const auto cameraDown = tvecs[i][1];
            const auto cameraForward = tvecs[i][2];

            auto dir = tf2::quatRotate(tf2::Quaternion(vec3::Right, M_PI / 4),
                Vector3(tvecs[i][0], -tvecs[i][2], tvecs[i][1]));

            dir = tf2::quatRotate(tf2::Quaternion(vec3::Up, robotPose.rotation), dir);

            markerArray.markers.push_back(Viz::ToDirection(Vector3(robotPose.position.x, robotPose.position.y, 16.5), dir, "map"));

            const double floorDown = cameraForward * std::sin(cameraPitchToFloor) + cameraDown * std::cos(cameraPitchToFloor);

            if (floorDown <= 1e-6)
                continue;

            const double scaleToFloor = cameraHeight / floorDown;

            const double robotLocalRight = cameraRight * scaleToFloor;
            const double robotLocalForward = (cameraForward * std::cos(cameraPitchToFloor) - cameraDown * std::sin(cameraPitchToFloor)) * scaleToFloor;

            const auto forward = robotPose.forward;

            const auto right = Vector2(
                std::cos(robotPose.rotation - M_PI * 0.5),
                std::sin(robotPose.rotation - M_PI * 0.5));

            const double worldX = robotPose.position.x + right.x * robotLocalRight + forward.x * robotLocalForward;

            const double worldY = robotPose.position.y + right.y * robotLocalRight + forward.y * robotLocalForward;

            const auto worldPosition = Vector2(worldX, worldY);
            //_mappingEngine->GetCell(Vector2(worldX, worldY))->GetWorldPosition();

            geometry_msgs::msg::Pose p;

            p.position.x = worldPosition.x;
            p.position.y = worldPosition.y;
            p.position.z = 0.0;

            p.orientation.x = 0.0;
            p.orientation.y = 0.0;
            p.orientation.z = 0.0;
            p.orientation.w = 1.0;

            visualization_msgs::msg::Marker positionMarker;
            positionMarker.header.stamp = stamp;
            positionMarker.header.frame_id = "map";
            positionMarker.ns = "aruco_position";
            positionMarker.id = static_cast<int>(i * 2 + 1);
            positionMarker.type = visualization_msgs::msg::Marker::SPHERE;
            positionMarker.action = visualization_msgs::msg::Marker::ADD;
            positionMarker.pose = p;
            positionMarker.scale.x = 0.08;
            positionMarker.scale.y = 0.08;
            positionMarker.scale.z = 0.08;
            positionMarker.color.r = 0.0f;
            positionMarker.color.g = 1.0f;
            positionMarker.color.b = 0.0f;
            positionMarker.color.a = 1.0f;
            markerArray.markers.push_back(positionMarker);

            visualization_msgs::msg::Marker textMarker;
            textMarker.header.stamp = stamp;
            textMarker.header.frame_id = "map";
            textMarker.ns = "aruco_text";
            textMarker.id = static_cast<int>(i * 2 + 2);
            textMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            textMarker.action = visualization_msgs::msg::Marker::ADD;
            textMarker.pose = p;
            textMarker.pose.position.z = 0.18;
            textMarker.scale.z = 0.14;
            textMarker.color.r = 1.0f;
            textMarker.color.g = 1.0f;
            textMarker.color.b = 1.0f;
            textMarker.color.a = 1.0f;
            textMarker.text = std::to_string(ids[i]);
            markerArray.markers.push_back(textMarker);

            cv::aruco::drawAxis(
                frame,
                _cameraMatrix,
                _distanceCoefficients,
                rvecs[i],
                tvecs[i],
                0.03);
        }
    }

    _markerPublisher->publish(markerArray);

    cv::flip(frame, frame, -1);

    auto outMessage = cv_bridge::CvImage(
        msg->header,
        "bgr8",
        frame)
                          .toImageMsg();

    _debugPublisher->publish(*outMessage);
}

void ArucoDetectionEngine::OnMappingEngineStateChange(MappingEngineStateChangeEvent event)
{
}

}
