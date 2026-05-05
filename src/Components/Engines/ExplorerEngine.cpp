#include "ExplorerEngine.hpp"
#include "Viz/Marker.hpp"
#include <stdexcept>

using namespace std;
using namespace Manhattan::nav;

namespace Manhattan::Core {
ExplorerEngine::ExplorerEngine(const App& app)
    : RosEngine(app, "explorer")
    , _grid(0, 0, 0)
    , _map(0, 0, 0)
{
    _mapping = app.GetComponent<MappingEngine>();
    _navigatorController = app.GetComponent<NavigatorEngine>();
}

void ExplorerEngine::OnEnable()
{
    _state = ExplorerState::Exploring;

    _mapSubscription = create_subscription<nav_msgs::msg::OccupancyGrid>("slam/grid_thinned", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->OnMap(msg);
    });

    _markerPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("explorer/markers", 1);

    _timer = create_wall_timer(100ms, [this] { Update(); });

    _publishTimer = create_wall_timer(100ms, [this] {
       Publish();
    });

}

void ExplorerEngine::OnDisable()
{
    _timer.reset();
    _mapSubscription.reset();
    _markerPublisher.reset();
}

ExplorerEngine::ExplorerResult ExplorerEngine::Explore(const Vector2 &inDirection, const Vector2Int startCell) const
{
    std::map<Vector2Int, Vector2Int> previous;
    std::queue<Vector2Int> queue;
    std::set<Vector2Int> visited;

    queue.push(startCell);
    visited.insert(startCell);
    std::optional<Vector2Int> frontier = std::nullopt;

    const auto dirNorm = inDirection.Normalized();
    auto dirs = Vector2Int::EightDirections();

    // Sort directions to prioritize inDirection
    ranges::sort(dirs, [&](const auto& a, const auto& b) {
        const Vector2 va = Vector2(a).Normalized();
        const Vector2 vb = Vector2(b).Normalized();

        const float da = Vector2::Dot(va, dirNorm);
        const float db = Vector2::Dot(vb, dirNorm);

        return da > db;
    });

    while (!queue.empty()) {
        Vector2Int current = queue.front(); queue.pop();

        int neighbourCount = 0;
        for (auto dir : dirs) {
            Vector2Int neighbor = current + dir;
            if (neighbor.x < 0 || neighbor.x >= _grid.width() ||
                neighbor.y < 0 || neighbor.y >= _grid.height())
                continue;

            if (!Walkable(_grid[neighbor]))
                continue;

            neighbourCount++;

            if (visited.contains(neighbor))
                continue;

            previous[neighbor] = current;
            queue.push(neighbor);
            visited.insert(neighbor);
        }

        if (neighbourCount == 2 || neighbourCount == 0) continue;

        frontier = current;
        break;
    }

    const auto target = frontier;

    std::vector<Vector2> path;
    if (!frontier)
        return ExplorerResult{ target, path };

    Vector2Int current = *frontier;

    while (current != startCell) {
        path.push_back(_mapping->GridToWorld(current));

        const auto it = previous.find(current);
        if (it == previous.end())
            return ExplorerResult{ target, {} };

        current = it->second;
    }

    ranges::reverse(path);
    return ExplorerResult{ target, path };
}

void ExplorerEngine::OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
    auto grid = viz::nav::ToOccupancyGrid(*msg, 50);
    _grid = grid;
    _map = GridMap(grid.width(), grid.height(), grid.resolution());

    Update();
}

std::optional<Vector2Int> ExplorerEngine::ClosestOnThinnedMap(const Vector2& pos) const
{
    const auto intPos = _mapping->WorldToGrid(pos);

    if (Walkable(_grid[intPos]))
        return intPos;

    std::queue<Vector2Int> q;
    std::set<Vector2Int> visited;

    q.push(intPos);
    visited.insert(intPos);

    while (!q.empty()) {
        auto cell = q.front(); q.pop();

        for (auto direction : Vector2Int::EightDirections()) {
            const auto neighbour = cell + direction;

            if (neighbour.x >= _grid.width() || neighbour.x < 0 || neighbour.y >= _grid.height() || neighbour.y < 0) continue;
            if (visited.contains(neighbour)) continue;

            if (Walkable(_grid[neighbour]))
                return neighbour;

            q.push(neighbour);
            visited.insert(neighbour);
        }
    }

    return std::nullopt;
}

void ExplorerEngine::Update()
{
    if (_grid.size() == 0 || !_navigatorController->IsInDestination())
        return;

    switch (_state) {
    case ExplorerState::Idle:
        break;

    case ExplorerState::Exploring: {
        const auto pose = _mapping->CurrentPose();

        if (!_currentTarget) {
            _currentTarget = ClosestOnThinnedMap(pose.position);

            if (!_currentTarget)
                return;
        }

        auto result = Explore(pose.forward, _currentTarget.value());
        _currentTarget = result.target;
        _navigatorController->SetPath(result.path);
        break;
    }
    case ExplorerState::Returning:
        _state = ExplorerState::Idle;
        break;

    default:
        throw std::out_of_range("Invalid ExplorerState");
    }
}

void ExplorerEngine::Publish() const
{
    auto markers = viz::marker::MarkerArrayBuilder();

    markers.add(viz::marker::clear("map"));

    if (_currentTarget != std::nullopt) {
        const auto worldPoint = _mapping->GridToWorld(_currentTarget.value()).ToTf2();

        markers.add(viz::marker::point(worldPoint, "map"));
    }

    _markerPublisher->publish(markers.array);
}

} // namespace Manhattan::Core
