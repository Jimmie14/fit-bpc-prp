#pragma once

#include "ArucoDetectionEngine.hpp"
#include "MappingEngine.hpp"
#include "Nav/GridMap.hpp"
#include "NavigatorEngine.hpp"
#include "Common/RosEngine.hpp"
#include "Viz/Grid.hpp"

#include <memory>
#include <vector>

namespace Manhattan::core {
enum class ExplorerState {
    Idle,
    Exploring,
    Returning
};

class ExplorerEngine : public RosEngine {
public:
    explicit ExplorerEngine(const App& app);

    void Update();

protected:
    void OnEnable() override;

    void OnDisable() override;

private:
    struct ExplorerResult {
        std::optional<Vector2i> target;
        std::vector<Vector3> path;
    };

    Grid<bool> _grid;
    GridMap _map;

    bool _inJunction = false;
    bool _reverse = false;
    bool atDeadEnd = true;

    std::shared_ptr<MappingEngine> _mapping;

    ExplorerState _state = ExplorerState::Idle;
    std::optional<Vector2i> _currentTarget = std::nullopt;

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

    ExplorerResult Explore(const Vector3 &inDirection, Vector2i startCell);

    std::optional<Vector2i> PickFollowingDirection(const Vector2i& current, const vector<Vector2i>& ways, const Vector3& forward, const Vector3& preferred) const;

    std::pair<std::vector<Vector2i>, std::set<Vector2i>> GetCrossroadWays(std::set<Vector2i> visited, const Vector2i& start) const;

    std::optional<Vector2i> ClosestOnThinnedMap(const Vector3& position) const;

    Vector3 GetPreferredDirection() const;

    void Publish() const;

    void OnAruCode(CodeDetectedEvent aruCode);
};
} // namespace Manhattan::Core
