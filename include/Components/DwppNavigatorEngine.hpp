#pragma once

#include "Kinematics.hpp"
#include "LinearPath.hpp"
#include "Math/Pose.hpp"
#include "RosEngine.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace Manhattan::Core {
class DwppNavigatorEngine final : public RosEngine {
public:
    explicit DwppNavigatorEngine(const App& app);

protected:

    void OnEnable() override;
    void OnDisable() override;

private:
    TimerBase::SharedPtr _updateTimer;
    TimerBase::SharedPtr _debugTimer;

    Kinematics _kinematics;
    Pose _pose;
    Twist _twist;

    LinearPath _path;
    Vector2 _lookaheadPoint;
    std::mutex _lock;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _debugPublisher;

    void Update();
    double Evaluate(const Twist& twist) const;

    void PublishDebug();
};

}
