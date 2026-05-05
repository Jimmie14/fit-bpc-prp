#include "ExplorerEngine.hpp"
#include <stdexcept>

using namespace std;

namespace Manhattan::Core {
ExplorerEngine::ExplorerEngine(const App& app)
    : RosEngine(app, "explorer"), _grid(0, 0, 0)
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

    _timer = create_wall_timer(100ms, [this] { Update(); });
}

void ExplorerEngine::OnDisable()
{
    _timer.reset();
    _mapSubscription.reset();
}

std::vector<Vector2> ExplorerEngine::Explore(const Vector2 &inDirection, const Vector2Int startCell, std::optional<Vector2Int>& out)
{
    std::map<Vector2Int, Vector2Int> previous;
    std::queue<Vector2Int> queue;
    std::set<Vector2Int> visited;

    queue.push(startCell);
    std::optional<Vector2Int> frontier = std::nullopt;

    while (!queue.empty()) {
        Vector2Int current = queue.front(); queue.pop();
        visited.insert(current);

        int neighbourCount = 0;
        for (auto dir : Vector2Int::EightDirections()) {
            const auto neighbor = current + dir;

            if (visited.contains(neighbor))
                continue;

            if (neighbor.x >= _grid.width() || neighbor.x < 0 || neighbor.y >= _grid.height() || neighbor.y < 0)
                continue;

            if (!_grid[neighbor])
                continue;

            previous[neighbor] = current;

            queue.push(neighbor);
            neighbourCount++;
        }

        if (neighbourCount > 0) continue;

        frontier = current;
        break;
    }

    out = frontier;

    std::vector<Vector2> path;
    if (!frontier)
        return path;

    while (frontier.has_value() && frontier.value() != startCell) {
        path.push_back(_mapping->GridToWorld(frontier.value()));

        const auto it = previous.find(frontier.value());
        if (it == previous.end()) {
            return path;
        }

        frontier = it->second;
    }

    ranges::reverse(path);
    return path;
}

void ExplorerEngine::OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
    auto grid = viz::nav::ToOccupancyGrid(*msg, 50);
    _grid = grid;
}

std::optional<Vector2Int> ExplorerEngine::ClosestOnThinnedMap(const Vector2& pos) const
{
    const auto intPos = _mapping->WorldToGrid(pos);

    if (_grid[intPos])
        return intPos;

    std::queue<Vector2Int> q;
    std::set<Vector2Int> visited;

    q.push(intPos);

    while (!q.empty()) {
        auto cell = q.front(); q.pop();

        for (auto direction : Vector2Int::EightDirections()) {
            const auto neighbour = cell + direction;

            if (neighbour.x >= _grid.width() || neighbour.x < 0 || neighbour.y >= _grid.height() || neighbour.y < 0)
                continue;

            if (_grid[neighbour])
                return neighbour;

            if (!visited.contains(neighbour))
                q.push(neighbour);
        }

        visited.insert(cell);
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
        if (!_currentTarget)
            _currentTarget = ClosestOnThinnedMap(_mapping->CurrentPose().position);
        if (!_currentTarget)
            return;

        auto path = Explore(_currentTarget.value(), _currentTarget);
        _navigatorController->SetPath(path);
        break;
    }
    case ExplorerState::Returning:
        _state = ExplorerState::Idle;
        break;

    default:
        throw std::out_of_range("Invalid ExplorerState");
    }
}
} // namespace Manhattan::Core
