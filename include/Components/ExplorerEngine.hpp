#pragma once

#include "ArucoDetectionEngine.hpp"
#include "MappingEngine.hpp"
#include "Nav/GridMap.hpp"
#include "NavigatorEngine.hpp"
#include "RosEngine.hpp"
#include "Viz/Grid.hpp"

#include <memory>
#include <vector>

namespace Manhattan::Core {
enum class ExplorerState {
    Idle,
    Exploring,
    Returning
};

class ExplorerEngine : public RosEngine {
public:
    ExplorerEngine(const App& app);

    void Update();

    void OnEnable() override;

    void OnDisable() override;

private:
    struct ExplorerResult {
        std::optional<Vector2Int> target;
        std::vector<Vector3> path;
    };

    Grid<bool> _grid;
    GridMap _map;

    bool _inJunction = false;

    std::shared_ptr<MappingEngine> _mapping;
    std::shared_ptr<NavigatorEngine> _navigatorController;

    ExplorerState _state = ExplorerState::Idle;
    std::optional<Vector2Int> _currentTarget = std::nullopt;

    Vector3 _junctionEnterDirection;

    vector<Vector3> _path;
    vector<Vector3> _options;

    std::optional<CodeDetectedEvent> _exitCode = std::nullopt;
    std::optional<CodeDetectedEvent> _treasureCode = std::nullopt;

    Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _mapSubscription;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _markerPublisher;

    std::mutex _mutex;

    TimerBase::SharedPtr _startTimer;
    TimerBase::SharedPtr _timer;
    TimerBase::SharedPtr _publishTimer;

    ExplorerResult Explore(const Vector3 &inDirection, Vector2Int startCell);

    std::optional<Vector2Int> PickFollowingDirection(const Vector2Int& current, const vector<Vector2Int>& ways, const Vector3& forward, const Vector3& preferred) const;

    std::pair<std::vector<Vector2Int>, std::set<Vector2Int>> GetCrossroadWays(std::set<Vector2Int> visited, const Vector2Int& start) const;

    std::optional<Vector2Int> ClosestOnThinnedMap(const Vector3& position) const;

    Vector3 GetPreferredDirection() const;

    void Publish() const;

    void OnAruCode(CodeDetectedEvent aruCode);
};
} // namespace Manhattan::Core
