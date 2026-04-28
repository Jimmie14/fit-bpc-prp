#pragma once

#include "MappingEngine.hpp"
#include "NavigatorEngine.hpp"
#include "PoseMatcher.hpp"
#include "Vector2.hpp"

namespace Manhattan::Core {
class FollowerEngine final : public RosEngine {
public:
    explicit FollowerEngine(const App& app);

    void OnEnable() override;

    void OnDisable() override;

private:
    std::vector<RayHit> _rayHits;
    Vector2 _startPosition;

    double _fov;
    double _rayDistance;
    int _rayCount;

    double _avoidanceDistance;

    TimerBase::SharedPtr _timer;
    TimerBase::SharedPtr _initialTimer;

    std::shared_ptr<MappingEngine> _map;
    std::shared_ptr<NavigatorEngine> _navigator;

    Vector2 GetTarget(const Pose& pose) const;

    void FollowCorridor();

    void Update();
};
} // namespace Manhattan::Core
