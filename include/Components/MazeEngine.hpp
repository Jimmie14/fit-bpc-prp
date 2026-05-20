#pragma once

#include "ArucoDetectionEngine.hpp"
#include "Common/RosEngine.hpp"
#include "Math/Vec3.hpp"
#include "Nav/PcaFilter.hpp"
#include "NavigatorEngine.hpp"

namespace Manhattan::core {

enum class NavState {
    FOLLOW_CORRIDOR,
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
    struct WayPoint {
        std::optional<CodeDetectedEvent> exitCode = std::nullopt;
        std::optional<CodeDetectedEvent> treasureCode = std::nullopt;
    };

    std::shared_ptr<NavigatorEngine> _navigator;
    std::shared_ptr<MappingEngine> _mapping;

    std::vector<WayPoint> _waypoints;
    // std::optional<CodeDetectedEvent> _exitCode = std::nullopt;
    // std::optional<CodeDetectedEvent> _treasureCode = std::nullopt;

    std::chrono::steady_clock::time_point _lastTurn;
    std::chrono::steady_clock::time_point _lastUpdate;
    float _dt{};

    std::vector<std::tuple<Vector2, Vector2, bool>> _rays;

    Vector2 _center = Vector2::zero();
    Vector2 _heading = Vector2(vec3::Forward);

    TimerBase::SharedPtr _timer;
    TimerBase::SharedPtr _initialTimer;
    TimerBase::SharedPtr _publisherTimer;

    float _targetRotation = 0.f;

    Pid _headingPid;
    Pid _turnPid;

    // float _prevTurnError = 0;
    // float _turnIntegralError = 0;
    //
    // float _prevHeadingError = 0;

    std::mutex _mutex;
    NavState _state = NavState::FOLLOW_CORRIDOR;

    std::optional<PcaFitter::FittedLine> _leftWall = std::nullopt;
    std::optional<PcaFitter::FittedLine> _rightWall = std::nullopt;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher;
    Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _ledPublisher;

    void Update();

    void FollowCorridor();

    bool CalculateWalls();

    void PickDirection(bool left, bool forward, bool right);

    void ExecuteTurnState();

    void RecenterState();

    bool DirectionIsFree(TurnDirection side);

    float GetHeadingError();

    std::vector<RayHit> RayArc(float fov, TurnDirection side, float dst) const;

    std::optional<PcaFitter::FittedLine> FilterHitPoints(const std::vector<RayHit>& hits) const;

    TurnDirection ChooseDirection(bool left, bool forward, bool right);

    void OnAruCode(CodeDetectedEvent aruCode);

    void publishDebug() const;
    void publishLeds() const;
};

} // namespace Manhattan::Core