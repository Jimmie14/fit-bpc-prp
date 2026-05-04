#pragma once

#include "MappingEngine.hpp"
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
    struct Cell {
        bool value;
        bool visited;
    };

    std::vector<Vector2> Explore(Vector2Int startCell) const;

    void OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);

    std::optional<Vector2Int> ClosestOnThinnedMap(const Vector2& position) const;

    TimerBase::SharedPtr _timer;
    nav::Grid<Cell> _grid;

    std::shared_ptr<MappingEngine> _mapping;
    std::shared_ptr<NavigatorEngine> _navigatorController;

    ExplorerState _state = ExplorerState::Idle;
    // std::optional<Vector2Int> _startCell = std::nullopt;

    Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _mapSubscription;
};
} // namespace Manhattan::Core
