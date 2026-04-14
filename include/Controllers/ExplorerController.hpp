#pragma once

#include "BaseController.hpp"
#include "SlamController.hpp"
#include "NavigatorController.hpp" // Replace with your actual navigator header
#include <queue>
#include <vector>
#include <map>
#include <memory>

namespace Manhattan::Core
{
    enum class ExplorerState
    {
        Idle,
        Exploring,
        Returning
    };

    class ExplorerController : public BaseController
    {
    public:
        ExplorerController(const App& app);

        void Update();

        void OnEnable() override;

        void OnDisable() override;

    private:
        GridCell* Recenter(GridCell* cell) const;
        std::queue<GridCell*> Explore(GridCell* startCell) const;

        rclcpp::TimerBase::SharedPtr _timer;

        std::shared_ptr<SlamController> _slamController;
        std::shared_ptr<NavigatorController> _navigatorController;

        ExplorerState _state = ExplorerState::Idle;
        GridCell* _startCell = nullptr;
    };
}
