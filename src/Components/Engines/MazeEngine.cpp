#include "MazeEngine.hpp"

#include "Viz/Grid.hpp"
#include "Viz/Marker.hpp"

using namespace std;

namespace Manhattan::Core {

using namespace Manhattan::Nav;

MazeEngine::MazeEngine(const App& app)
    : RosEngine(app, "maze"), _thinned_map()
{
    _navigator = app.GetComponent<NavigatorEngine>();
    _mapping = app.GetComponent<MappingEngine>();
}

void MazeEngine::OnEnable() {
    _poseSubscription = create_subscription<geometry_msgs::msg::PoseStamped>("slam/pose", 1, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->OnPose(msg);
    });

    _mapSubscription = create_subscription<nav_msgs::msg::OccupancyGrid>("slam/grid_thinned", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->OnMap(msg);
    });

    _initialTimer = create_wall_timer(1s, [this]() {
        _initialTimer->cancel();
        _timer = create_wall_timer(100ms, [this] { Update(); });
    });
}

void MazeEngine::OnDisable() {
    _poseSubscription.reset();
    _mapSubscription.reset();

    _timer.reset();
    _initialTimer.reset();
}

void MazeEngine::OnPose(const geometry_msgs::msg::PoseStamped::SharedPtr& msg) const
{
    const auto pose = Pose::FromRosPoseMessage(msg->pose);
}

void MazeEngine::OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
    auto grid = Viz::ToOccupancyGrid(*msg, 50);
    _thinned_map = grid;
}

void MazeEngine::Update() {
    if (!_thinned_map)
        return;
    if (!_navigator->IsInDestination())
        return;

    if (_currentWayPoint == nullptr) {
        _currentWayPoint = std::make_shared<WayPoint>();

        _currentWayPoint->position = _mapping->CurrentPose().position;
        _currentWayPoint->connected = {};
        _currentWayPoint->visited = true;
    }

    const auto target = NextJunction(_currentWayPoint);
    if (target == nullptr)
        return;

    _currentWayPoint->visited = true;
    _currentWayPoint = target;

    const auto cell = _mapping->GetCell(target->position);
    _navigator->SetDestination(cell);
}



std::optional<Vector2Int> MazeEngine::ClosestOnThinnedMap(const Vector2& pos) {
    const auto intPos = _mapping->WorldToGrid(pos);

    std::queue<Vector2Int> q;
    std::set<Vector2Int> visited;

    q.push(intPos);

    while (!q.empty()) {
        auto cell = q.front(); q.pop();
        if (visited.contains(cell))
            continue;

        for (auto direction : Vector2Int::EightDirections()) {
            const auto neighbour = cell + direction;

            if (neighbour.x >= _thinned_map->width || neighbour.x < 0 || neighbour.y >= _thinned_map->height || neighbour.y < 0)
                continue;

            if ((*_thinned_map)[neighbour])
                return neighbour;

            q.push(neighbour);
        }

        visited.insert(cell);
    }

    return std::nullopt;
}

std::shared_ptr<MazeEngine::WayPoint> MazeEngine::NextJunction(std::shared_ptr<WayPoint> current) {
    std::queue<Vector2Int> q;
    std::set<Vector2Int> visited;

    const auto initPos = ClosestOnThinnedMap(current->position);
    if (!initPos) return nullptr;

    q.push(initPos.value());
    visited.insert(q.front());

    while (!q.empty()) {
        auto cell = q.front(); q.pop();
        int traversable_neighbors = 0;
        std::vector<Vector2Int> neighbors;
    }
}

} // namespace Manhattan::Core