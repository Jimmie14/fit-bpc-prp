#pragma once

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "Kinematics.hpp"
#include "MappingEngine.hpp"
#include "MotorDriver.hpp"
#include "Pid.hpp"
#include "RosEngine.hpp"
#include "LinearPath.hpp"

namespace Manhattan::Core {
class NavigatorEngine final : public RosEngine {
public:
    explicit NavigatorEngine(const App& app);

    void SetPath(const std::vector<Vector2>& path);

    [[nodiscard]] bool IsInDestination() const;
    void ClearPath();

    void SetDestination(GridCell* destination);

private:
    double _currentAngularVelocity = 0.0;
    double _currentLinearVelocity = 0.0;

    std::chrono::steady_clock::time_point _lastTime;

    Kinematics _kinematics;
    Pid _angularPid;

    TimerBase::SharedPtr _timer;
    std::shared_ptr<MotorDriver> _motor;
    std::shared_ptr<MappingEngine> _slam;

    LinearPath _path;
    Publisher<nav_msgs::msg::Path>::SharedPtr _pathPublisher;
    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _rayCastPublisher;

    std::vector<RayHit> RayCastAround(const Pose& pose) const;
    static Vector2 GetDirection(const std::vector<RayHit>& rayHits, const Pose& pose, const Vector2& desiredDirection);

    void PublishPath() const;
    void PublishRayCast(const std::vector<RayHit>& hits, const Pose& pose, const Vector2& desiredDirection) const;
    double GetLinearVelocity(const Pose& pose, double t, double minDistance, double delta) const;
    static double ClosestDistance(const std::vector<RayHit>& rayHits, const Pose& pose);

    void Update();

    void OnMappingEngineStateChange(MappingEngineStateChangeEvent event);
};

} // namespace Manhattan::Core
