#pragma once

#include "App.hpp"
#include "MappingEngine.hpp"
#include "Nav/GridMap.hpp"
#include "opencv2/aruco.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <sensor_msgs/msg/detail/compressed_image__struct.hpp>

namespace Manhattan::core {

using namespace Manhattan::nav;

struct CodeDetectedEvent {
    int id;
    Pose pose;
};

class ArucoDetectionEngine : public RosEngine {
public:
    ArucoDetectionEngine(const App& app);

    void OnEnable() override;

    void OnDisable() override;

private:
    struct Code {
        int id;
        std::pair<int, int> position;
        float theta;
    };

    TimerBase::SharedPtr _publishTimer;

    Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _imageSubscription;
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _poseSubscription;
    Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _mapSubscription;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _markerPublisher;
    Publisher<sensor_msgs::msg::Image>::SharedPtr _debugPublisher;

    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> _parameters;

    Pose _lastPose;
    GridMap _map;
    vector<Code> _codes = {};

    cv::Mat _cameraMatrix;
    cv::Mat _distanceCoefficients;

    void OnPose(const geometry_msgs::msg::PoseStamped::SharedPtr& msg);
    void OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);

    void OnImage(const sensor_msgs::msg::CompressedImage::SharedPtr& msg);

    void OnMappingEngineStateChange(MappingEngineStateChangeEvent event);

    void UpdateOrCreateCode(const int id, const std::pair<int, int>& position, const float rotation);

    void Publish() const;
};
}
