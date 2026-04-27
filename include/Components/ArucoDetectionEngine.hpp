#pragma once

#include "App.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

namespace Manhattan::Core {
class ArucoDetectionEngine : public RosEngine {
public:
    ArucoDetectionEngine(const App& app);

    void OnEnable() override;

    void OnDisable() override;

private:
    Subscription<sensor_msgs::msg::Image>::SharedPtr _imageSubscription;

    Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _posePublisher;
    Publisher<sensor_msgs::msg::Image>::SharedPtr _debugPublisher;


    void OnImage(const sensor_msgs::msg::Image::SharedPtr& msg);
};
}
