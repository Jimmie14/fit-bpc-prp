#pragma once

#include "MappingEngine.hpp"
#include "Nav/Grid.hpp"
#include "Nav/GridMap.hpp"
#include "NavigatorEngine.hpp"
#include "RosEngine.hpp"

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
    GridCell* Explore(GridCell* startCell) const;

    std::optional<Vector2Int> ClosestOnThinnedMap(const tf2::Vector3& pos) const;

    vector<tf2::Vector3> GetPath(Vector2Int target, const tf2::Vector3& currentPos) const;

    nav::Grid<bool> _grid;
    nav::GridMap _map;

    TimerBase::SharedPtr _timer;

    std::shared_ptr<MappingEngine> _mapping;
    std::shared_ptr<NavigatorEngine> _navigatorController;

    ExplorerState _state = ExplorerState::Idle;
    GridCell* _startCell = nullptr;
};
} // namespace Manhattan::Core
