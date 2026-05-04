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
    // _startCell = _mapping->GetCell(_mapping->CurrentPose().position);
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

std::vector<Vector2> ExplorerEngine::Explore(const Vector2Int startCell) const
{
    std::map<Vector2Int, Vector2Int> previous;
    std::vector<Vector2Int> queue;
    std::set<Vector2Int> visited;

    queue.push_back(startCell);
    std::optional<Vector2Int> frontier = std::nullopt;

    while (!queue.empty()) {
        Vector2Int current = queue.front();
        queue.erase(queue.begin());

        int neighbourCount = 0;
        for (auto dir : Vector2Int::EightDirections()) {
            const auto neighbor = current + dir;

            if (visited.contains(neighbor))
                continue;

            if (_grid[neighbor].visited)
                continue;

            if (!_grid[neighbor].value)
                continue;

            previous[neighbor] = current;

            queue.push_back(neighbor);
            neighbourCount++;
        }

        if (neighbourCount > 0) continue;

        frontier = current;
        break;
    }

    std::vector<Vector2> path;

    if (!frontier.has_value())
        return path;

    while (frontier.has_value() && frontier != startCell) {
        path.push_back(_mapping->GridToWorld(frontier.value()));

        const auto it = previous.find(frontier.value());
        if (it == previous.end()) {
            return path;
        }

        frontier = it->second;
    }

    return path;
}

void ExplorerEngine::OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
    auto grid = viz::nav::ToOccupancyGrid(*msg, 50);

    if (_grid.width() != grid.width() || _grid.height() != grid.height())
        _grid = nav::Grid<Cell>(grid.width(), grid.height(), grid.resolution());

    for (auto i = 0; i < grid.size(); i++) {
        auto value = !grid[i];
        auto gridCell = Cell{value,_grid[i].visited && value};

        _grid.set(i, gridCell);
    }
}

std::optional<Vector2Int> ExplorerEngine::ClosestOnThinnedMap(const Vector2& pos) const
{
    const auto intPos = _mapping->WorldToGrid(pos);

    std::queue<Vector2Int> q;
    std::set<Vector2Int> visited;

    q.push(intPos);

    bool any = false;
    while (!q.empty()) {
        auto cell = q.front(); q.pop();
        if (visited.contains(cell))
            continue;

        for (auto direction : Vector2Int::EightDirections()) {
            const auto neighbour = cell + direction;

            if (neighbour.x >= _grid.width() || neighbour.x < 0 || neighbour.y >= _grid.height() || neighbour.y < 0)
                continue;

            if (_grid[neighbour].value)
                return neighbour;

            if (!any && _grid[neighbour].value)
                any = true;

            q.push(neighbour);
        }

        if (any) {
            int i;
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
        auto startCell = ClosestOnThinnedMap(_mapping->CurrentPose().position);
        if (!startCell.has_value()) return;

        auto path = Explore(startCell.value());
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
