#pragma once

#include "MappingEngine.hpp"
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
    GridCell* Recenter(GridCell* cell) const;
    GridCell* Explore(GridCell* startCell) const;

    TimerBase::SharedPtr _timer;

    std::shared_ptr<MappingEngine> _slamController;
    std::shared_ptr<NavigatorEngine> _navigatorController;

    ExplorerState _state = ExplorerState::Idle;
    GridCell* _startCell = nullptr;
};
} // namespace Manhattan::Core
