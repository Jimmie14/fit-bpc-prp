#include "Components/ManualPathPlannerEngine.hpp"

#include "App.hpp"
#include "Components/MapThinningUnit.hpp"
#include "Messages/Nav.hpp"

namespace Manhattan::core {
ManualPathPlannerEngine::ManualPathPlannerEngine(const App& app)
    : RosEngine(app, "manual_path_planner")
{
    app.events->Subscribe<messages::MazeGraphPublishEvent>([this](const auto& msg) {
        std::lock_guard guard(_lock);

        _graph = msg.graph;
    });

    app.events->Subscribe<messages::RobotPoseEvent>([this](const auto& event) {
        std::lock_guard guard(_lock);

        _pose = event.pose;
    });

    app.events->Subscribe<ThinnedMapEvent>([this](const auto& event) {
        std::lock_guard guard(_lock);

        _map = GridMap(event.grid);

        planPath();
    });
}

void ManualPathPlannerEngine::OnEnable()
{
    _pointSubscription = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 2, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            if (msg->header.frame_id != "map") return;
            if (_map.empty()) return;

            const Vector2i coord = _map.worldToCoord(Vector3(msg->point.x, msg->point.y, 0));

            _target = _graph.findClosestPosition(coord, 20);
        });
}

void ManualPathPlannerEngine::OnDisable()
{
    _pointSubscription.reset();
}

void ManualPathPlannerEngine::planPath()
{
    if (_map.empty()) return;
    if (!_target.has_value()) return;

    if (!_graph.containsEdge(_target->from, _target->to)) {
        _target = nullopt;
        return;
    }

    const Vector2i coord = _map.worldToCoord(_pose.position.toTf2());
    const auto node = _graph.findClosestPosition(coord, 20);
    if (!node.has_value()) return;

    const auto path = _graph.calculatePath(node.value(), _target.value());
    if (!path.has_value()) return;

    vector<Vector3> worldPath;
    worldPath.reserve(path.value().size());

    for (auto point : path.value()) {
        worldPath.push_back(_map.coordToWorld(point));
    }

    _app.events->Publish<messages::RobotFollowPathEvent>({ worldPath });
}

}