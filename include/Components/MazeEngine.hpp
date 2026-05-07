#pragma once

#include "ArucoDetectionEngine.hpp"
#include "Nav/Grid.hpp"
#include "NavigatorEngine.hpp"
#include "NavigatorGraphBuilder.hpp"
#include "RosEngine.hpp"

namespace Manhattan::Core {

enum class NavState {
    FOLLOW_CORRIDOR,
    APPROACH_INTERSECTION,
    TURNING,
    RECENTER
};

enum class TurnDirection {
    FORWARD,
    LEFT,
    RIGHT,
    BACK
};

class MazeEngine final : public RosEngine {
public:
    explicit MazeEngine(const App& app);

    void OnEnable() override;
    void OnDisable() override;

private:
    std::shared_ptr<NavigatorEngine> _navigator;
    std::shared_ptr<MappingEngine> _mapping;

    TimerBase::SharedPtr _timer;
    TimerBase::SharedPtr _initialTimer;

    float _targetRotation = 0.f;

    float _prevError = 0;
    float _prevTurnError = 0;

    std::mutex _mutex;
    NavState _state = NavState::FOLLOW_CORRIDOR;

    TurnDirection _preferredDirection = TurnDirection::LEFT;
    TurnDirection _currentDecision = TurnDirection::LEFT;

    std::optional<CodeDetectedEvent> _exitCode = std::nullopt;
    std::optional<CodeDetectedEvent> _treasureCode = std::nullopt;

    std::chrono::steady_clock::time_point _lastDecision;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _graphPublisher;

    void Update();

    void FollowCorridor();

    void StartDecision(bool left, bool forward, bool right);

    void ExecuteTurn();

    void Recenter();

    float GetLeftWallDistance();

    float GetRightWallDistance();

    float GetFrontWallDistance();

    float GetBehindWallDistance();

    TurnDirection ChooseDirection(bool left, bool forward, bool right);

    void OnAruCode(CodeDetectedEvent aruCode);
};

} // namespace Manhattan::Core