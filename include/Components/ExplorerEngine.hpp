#pragma once

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
        std::vector<tf2::Vector3> path;
    };

    nav::Grid<bool> _grid;
    nav::GridMap _map;

    std::shared_ptr<MappingEngine> _mapping;
    std::shared_ptr<NavigatorEngine> _navigatorController;

    ExplorerState _state = ExplorerState::Idle;
    std::optional<Vector2Int> _currentTarget = std::nullopt;
    vector<tf2::Vector3> _path;

    Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _mapSubscription;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _markerPublisher;


    TimerBase::SharedPtr _startTimer;
    TimerBase::SharedPtr _timer;
    TimerBase::SharedPtr _publishTimer;

    ExplorerResult Explore(const tf2::Vector3 &inDirection, Vector2Int startCell) const;
    std::pair<std::vector<Vector2Int>, std::set<Vector2Int>> GetCrossroadWays(const Vector2Int& start, vector<Vector2Int>& directions) const;

    std::optional<Vector2Int> ClosestOnThinnedMap(const tf2::Vector3& position) const;

    void Publish() const;
};
} // namespace Manhattan::Core
