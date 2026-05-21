#include "Components/MazePathPlannerEngine.hpp"

#include "App.hpp"
#include "Components/MapThinningUnit.hpp"
#include "Messages/Nav.hpp"

namespace Manhattan::core {

MazePathPlannerEngine::MazePathPlannerEngine(const App& app)
    : RosEngine(app, "maze_path_planner")
{
    app.events->Subscribe<messages::MazeGraphPublishEvent>([this](const auto& event) {
        std::lock_guard guard(_lock);

        _graph.update(event.graph);
    });

    app.events->Subscribe<messages::RobotPoseEvent>([this](const auto& event) {
        std::lock_guard guard(_lock);

        _pose = event.pose;
    });

    app.events->Subscribe<ThinnedMapEvent>([this](const auto& event) {
        std::lock_guard guard(_lock);

        _map = GridMap(event.grid);
    });
}

void MazePathPlannerEngine::OnEnable()
{
    _timer = create_wall_timer(500ms, [this] {
        std::lock_guard guard(_lock);

        planPath();
    });
}

void MazePathPlannerEngine::OnDisable()
{
    _timer->reset();
}

void MazePathPlannerEngine::planPath()
{
    if (_map.empty()) return;

    if (!_initialized) {
        const auto startNode = _graph.findClosestNode(_map.worldToCoord(_pose.position.toTf2()), 20);
        if (!startNode.has_value()) return;

        _lastNodeId = startNode.value();
        _currentNodeId = startNode.value();

        const auto neighbours = _graph.getNeighbours(_currentNodeId);
        if (neighbours.empty()) return;

        _nextNodeId = neighbours.front();
        _initialized = true;
    }

    if (!_graph.nodes().contains(_lastNodeId) || !_graph.nodes().contains(_currentNodeId)) {
        _initialized = false;
        return;
    }

    if (!_graph.nodes().contains(_nextNodeId) || _graph.existEdge(_currentNodeId, _nextNodeId) == false) {

        const auto neighbours = _graph.getNeighbours(_currentNodeId);

        _nextNodeId = neighbours.front();
    }

    if (_graph.findClosestNode(_map.worldToCoord(_pose.position.toTf2()), 20) == _nextNodeId) {
        _lastNodeId = _currentNodeId;
        _currentNodeId = _nextNodeId;

        const auto next = _graph.followEdge(_lastNodeId, _currentNodeId, M_PI_2);
        if (!next.has_value()) return;

        _nextNodeId = next.value();
    }

    std::cout << "last     : " << _lastNodeId << std::endl;
    std::cout << "current  : " << _currentNodeId << std::endl;
    std::cout << "next     : " << _nextNodeId << std::endl;

}

}