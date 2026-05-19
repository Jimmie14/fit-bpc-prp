#pragma once

#include "Math/LinearPath.hpp"
#include "Kinematics/Kinematics.hpp"
#include "Math/Pose.hpp"
#include "Common/RosEngine.hpp"
#include "Config/DwppConfig.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace Manhattan::core {

using namespace Manhattan::math;

class DwppNavigatorEngine final : public RosEngine {
public:
    explicit DwppNavigatorEngine(const App& app);

protected:

    void OnEnable() override;
    void OnDisable() override;

private:
    config::DwppConfig _config;

    TimerBase::SharedPtr _updateTimer;
    TimerBase::SharedPtr _debugTimer;

    kinematics::DifferentialDriveKinematics _kinematics;
    kinematics::DifferentialDriveOdometry _odometry;
    Pose _pose;
    Twist _twist;

    LinearPath _path;
    Vector2 _lookaheadPoint;
    std::mutex _lock;

    std::vector<std::pair<tf2::Vector3, double>> _debugSimulations;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _debugPublisher;

    void Update();
    double Evaluate(const Twist& twist);

    void PublishDebug();
};

}
