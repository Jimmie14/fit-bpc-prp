#pragma once

#include "App.hpp"
#include "MappingEngine.hpp"
#include "opencv2/aruco.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <sensor_msgs/msg/detail/compressed_image__struct.hpp>

namespace Manhattan::Core {
class ArucoDetectionEngine : public RosEngine {
public:
    ArucoDetectionEngine(const App& app);

    void OnEnable() override;

    void OnDisable() override;

private:
    shared_ptr<MappingEngine> _mappingEngine;

    Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _imageSubscription;
    Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraInfoSubscription;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _markerPublisher;
    Publisher<sensor_msgs::msg::Image>::SharedPtr _debugPublisher;

    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> _parameters;

    cv::Mat _cameraMatrix;
    cv::Mat _distanceCoefficients;

    bool _hasCameraInfo = false;

    void OnCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr& msg);
    void OnImage(const sensor_msgs::msg::CompressedImage::SharedPtr& msg);

    void OnMappingEngineStateChange(MappingEngineStateChangeEvent event);
};
}
