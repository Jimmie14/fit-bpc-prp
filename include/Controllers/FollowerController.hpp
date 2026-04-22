#pragma once

#include "../Common/RosConnector.hpp"
#include "MappingEngine.hpp"
#include "NavigatorController.hpp"
#include "PoseMatcher.hpp"
#include "Vector2.hpp"

namespace Manhattan::Core {
class FollowerController final : public RosConnector {
public:
    explicit FollowerController(const App& app);

    void OnEnable() override;

    void OnDisable() override;

private:
    std::vector<RayHit> _rayHits;
    Vector2 _startPosition;

    double _fov;
    double _rayDistance;
    int _rayCount;

    double _avoidanceDistance;

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::TimerBase::SharedPtr _initialTimer;

    std::shared_ptr<MappingEngine> _map;
    std::shared_ptr<NavigatorController> _navigator;

    Vector2 GetTarget(const Pose& pose) const;

    void FollowCorridor();

    void Update();
};
} // namespace Manhattan::Core
