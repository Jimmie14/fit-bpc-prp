#pragma once

#include "../Common/RosConnector.hpp"
#include "MappingEngine.hpp"
#include "NavigatorController.hpp"
#include <map>
#include <memory>
#include <queue>
#include <vector>

namespace Manhattan::Core {
enum class ExplorerState {
    Idle,
    Exploring,
    Returning
};

class ExplorerController : public RosConnector {
public:
    ExplorerController(const App& app);

    void Update();

    void OnEnable() override;

    void OnDisable() override;

private:
    GridCell* Recenter(GridCell* cell) const;
    GridCell* Explore(GridCell* startCell) const;

    rclcpp::TimerBase::SharedPtr _timer;

    std::shared_ptr<MappingEngine> _slamController;
    std::shared_ptr<NavigatorController> _navigatorController;

    ExplorerState _state = ExplorerState::Idle;
    GridCell* _startCell = nullptr;
};
} // namespace Manhattan::Core
