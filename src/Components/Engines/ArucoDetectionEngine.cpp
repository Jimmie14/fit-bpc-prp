#include "ArucoDetectionEngine.hpp"

#include "Math/Vec3.hpp"
#include "Viz/Marker.hpp"

#include <cv_bridge/cv_bridge.h>
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
    , _map(0, 0, 0.0f)
{
    _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    _parameters = cv::aruco::DetectorParameters::create();

    // height: 16.5cm, pi/4 rad
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
    _imageSubscription = create_subscription<sensor_msgs::msg::CompressedImage>(
        imageTopic, 1, [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            this->OnImage(msg);
        });

    _poseSubscription = create_subscription<geometry_msgs::msg::PoseStamped>("slam/pose", 1, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->OnPose(msg);
    });

    _mapSubscription = create_subscription<nav_msgs::msg::OccupancyGrid>("slam/grid", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->OnMap(msg);
    });

    _markerPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("aruco/markers", 1);
    _debugPublisher = create_publisher<sensor_msgs::msg::Image>("aruco/debug_image", 1);

    _publishTimer = create_wall_timer(1s, [this] {
        this->Publish();
    });
}

void ArucoDetectionEngine::OnDisable()
{
    _imageSubscription.reset();
    _poseSubscription.reset();
    _mapSubscription.reset();

    _markerPublisher.reset();
    _debugPublisher.reset();
}

void ArucoDetectionEngine::OnPose(const geometry_msgs::msg::PoseStamped::SharedPtr& msg)
{
    _lastPose = Pose::FromRosPoseMessage(msg->pose);
}

void ArucoDetectionEngine::OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
    _map = GridMap(msg->info.width, msg->info.height, msg->info.resolution);
    _mapSubscription.reset();
}


void ArucoDetectionEngine::OnImage(const sensor_msgs::msg::CompressedImage::SharedPtr& msg)
{
    if (_map.width() == 0) return;

    const auto frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    if (frame.empty())
        return;

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

    cv::aruco::detectMarkers(gray, _dictionary, corners, ids, _parameters);

    const auto stamp = this->now();

    const auto cameraPosition = _lastPose.position.ToTf2() + vec3::Up * cameraHeight;
    const auto robotRotation = _lastPose.rotation;

    if (!ids.empty()) {
        std::vector<cv::Vec3d> rVectors, tVectors;

        cv::aruco::estimatePoseSingleMarkers(
            corners,
            markerSize,
            _cameraMatrix,
            _distanceCoefficients,
            rVectors,
            tVectors);

        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        for (size_t i = 0; i < ids.size(); i++) {
            auto dir = tf2::quatRotate(tf2::Quaternion(vec3::Right, M_PI / 4),
                Vector3(tVectors[i][0], -tVectors[i][2], tVectors[i][1]));

            dir = tf2::quatRotate(tf2::Quaternion(vec3::Up, robotRotation + M_PI), dir);

            if (abs(dir.z()) <= 1e-6) continue;

            // auto t = -cameraPosition.z() / dir.z();
            //
            // const auto worldPoint = Vector3(cameraPosition.x() + t * dir.x(), cameraPosition.y() + t * dir.y(), 0.0);

            const auto worldPoint = Vector3(cameraPosition.x() + dir.x(), cameraPosition.y() + dir.y(), 0.0f);
            const auto gridCoord = _map.worldToCoord(worldPoint);

            auto& code = GetClosestOrCreateCode(gridCoord, 0.0f);

            code.id = ids[i];
            code.position = gridCoord;

            cv::aruco::drawAxis(
                frame,
                _cameraMatrix,
                _distanceCoefficients,
                rVectors[i],
                tVectors[i],
                0.03);
        }
    }

    cv::flip(frame, frame, -1);

    const auto outMessage = cv_bridge::CvImage(
        msg->header, "bgr8", frame).toImageMsg();

    _debugPublisher->publish(*outMessage);
}

void ArucoDetectionEngine::OnMappingEngineStateChange(MappingEngineStateChangeEvent event)
{
    _codes.clear();
}

ArucoDetectionEngine::Code& ArucoDetectionEngine::GetClosestOrCreateCode(const std::pair<int, int>& position, const float theta)
{
    for (auto& code : _codes) {
        if (code.position.first != position.first) continue;
        if (code.position.second != position.second) continue;

        return code;
    }

    _codes.push_back(Code { 0, position, theta });

    return _codes.back();
}

void ArucoDetectionEngine::Publish() const
{
    if (_map.width() == 0) return;

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(viz::marker::clear("map"));

    for (auto code : _codes) {
        const auto worldPoint = _map.coordToWorld(code.position);

        markers.markers.push_back(viz::marker::point(worldPoint, "map"));
        markers.markers.push_back(viz::marker::text(worldPoint, std::to_string(code.id), "map"));
    }

    _markerPublisher->publish(markers);
}

}
