#pragma once

#include "ArucoDetectionEngine.hpp"
#include "Nav/Grid.hpp"
#include "Nav/PcaFilter.hpp"
#include "NavigatorEngine.hpp"
#include "RosEngine.hpp"

namespace Manhattan::Core {

enum class NavState {
    FOLLOW_CORRIDOR,
    APPROACH_INTERSECTION,
    TURNING
};

enum class TurnDirection {
    FORWARD,
    LEFT,
    RIGHT
};

class MazeEngine final : public RosEngine {
public:
    explicit MazeEngine(const App& app);

    void OnEnable() override;
    void OnDisable() override;

private:
    std::shared_ptr<NavigatorEngine> _navigator;
    std::shared_ptr<MappingEngine> _mapping;

    std::chrono::steady_clock::time_point _lastUpdate;
    float _dt;

    TimerBase::SharedPtr _timer;
    TimerBase::SharedPtr _initialTimer;
    TimerBase::SharedPtr _publisherTimer;

    float _targetRotation = 0.f;

    float _prevError = 0;
    float _prevTurnError = 0;
    float _prevCenterError = 0;
    float _prevHeadingError = 0;

    std::mutex _mutex;
    NavState _state = NavState::FOLLOW_CORRIDOR;

    std::optional<CodeDetectedEvent> _exitCode = std::nullopt;
    std::optional<CodeDetectedEvent> _treasureCode = std::nullopt;

    std::optional<PcaFitter::FittedLine> _leftWall = std::nullopt;
    std::optional<PcaFitter::FittedLine> _rightWall = std::nullopt;
    std::optional<PcaFitter::FittedLine> _frontWall = std::nullopt;

    std::chrono::steady_clock::time_point _lastDecision;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher;

    void Update();

    void FollowCorridor();

    void CalculateWalls();

    void StartDecision(bool left, bool forward, bool right);

    void ExecuteTurn();

    float GetWallDistance(TurnDirection side) const;

    std::vector<RayHit> RayArc(float fov, TurnDirection side, float dst) const;

    std::optional<PcaFitter::FittedLine> FilterHitPoints(const std::vector<RayHit>& hits) const;

    TurnDirection ChooseDirection(bool left, bool forward, bool right);

    void OnAruCode(CodeDetectedEvent aruCode);

    void PublishWalls() const;
};

} // namespace Manhattan::Core