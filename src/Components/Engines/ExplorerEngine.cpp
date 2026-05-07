#include "ExplorerEngine.hpp"

#include "MapThinningUnit.hpp"

#include <algorithm>

using namespace std;

namespace Manhattan::Core {
ExplorerEngine::ExplorerEngine(const App& app)
    : RosEngine(app, "explorer"), _grid(0, 0, 0), _map(0, 0, 0)
{
    _mapping = app.GetComponent<MappingEngine>();
    _navigatorController = app.GetComponent<NavigatorEngine>();

    _app.Events->Subscribe<ThinnedMapEvent>([this](const ThinnedMapEvent& event) {
        _grid = event.grid;
        _map = GridMap(_grid.width(), _grid.height(), _grid.resolution());
   });
}

void ExplorerEngine::OnEnable()
{
    _startCell = _mapping->GetCell(_mapping->CurrentPose().position);
    _state = ExplorerState::Exploring;

    _timer = create_wall_timer(100ms, [this] { Update(); });
}

void ExplorerEngine::OnDisable()
{
    _timer.reset();
}

GridCell* ExplorerEngine::Explore(GridCell* startCell) const
{
    if (startCell == nullptr)
        return nullptr;

    std::map<GridCell*, double> distances;
    std::map<GridCell*, GridCell*> previous;
    std::vector<GridCell*> queue;

    distances[startCell] = 0.0;
    queue.push_back(startCell);
    GridCell* frontier = nullptr;

    while (!queue.empty()) {
        ranges::sort(queue, [&distances](GridCell* a, GridCell* b) { return distances[a] < distances[b]; });

        GridCell* current = queue.front();
        queue.erase(queue.begin());

        bool isBorder = false;

        for (GridCell* neighbor : _mapping->GetNeighbors(current)) {
            double alt = distances[current] + neighbor->GetCost();

            auto distIt = distances.find(neighbor);
            if (distIt != distances.end() && alt >= distIt->second)
                continue;

            distances[neighbor] = alt;
            previous[neighbor] = current;

            if (ranges::find(queue, neighbor) != queue.end())
                continue;

            if (neighbor->IsUnknown()) {
                isBorder = true;
                continue;
            }

            if (neighbor->IsOccupied())
                continue;

            queue.push_back(neighbor);
        }

        if (!isBorder)
            continue;

        frontier = current;
        break;
    }

    return frontier;
}


static bool Walkable(const bool value)
{
    return value;
}

std::optional<Vector2Int> ExplorerEngine::ClosestOnThinnedMap(const Vector3& pos) const
{
    const auto gridPos = _map.worldToCoord(pos);
    const auto intPos = Vector2Int(gridPos.first, gridPos.second);

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

vector<Vector3> ExplorerEngine::GetPath(const Vector2Int target, const Vector3& currentPos) const
{
    struct Cell {
        Vector2Int position;
        Cell* from;
    };

    const auto start = ClosestOnThinnedMap(currentPos);

    vector<Vector3> path;

    if (!start.has_value())
        return path;

    std::queue<Cell> toVisit;
    std::set<Vector2Int> visited;
    std::optional<Cell> frontier = std::nullopt;

    toVisit.push({start.value(), nullptr});
    visited.insert(start.value());

    auto minDistance = std::numeric_limits<double>::max();

    while (!toVisit.empty()) {
        auto current = toVisit.front(); toVisit.pop();

        if (Vector2::Distance(Vector2(current.position), Vector2(target)) < minDistance) {
            frontier = current;
        }

        for (auto dir : Vector2Int::EightDirections()) {
            Vector2Int neighbour = current.position + dir;
            if (!_grid.inBounds(neighbour.x, neighbour.y)) continue;

            if (!Walkable(_grid[neighbour])) continue;
            if (visited.contains(neighbour)) continue;


            toVisit.push(Cell{neighbour, &current});
            visited.insert(neighbour);
        }
    }

    if (!frontier.has_value())
        return path;

    auto current = frontier.value();
    while (current.from) {
        path.push_back(_map.coordToWorld(current.position));
        current = *current.from;
    }

    return path;
}


void ExplorerEngine::Update()
{
    if (_grid.size() == 0 || !_navigatorController->IsInDestination())
        return;

    const auto pose = _mapping->CurrentPose();
    GridCell* startCell = _mapping->GetCell(pose.position);

    auto target = Explore(startCell);
    if (target == nullptr)
        return;

    _navigatorController->SetDestination(target);
    return;

    const auto path = GetPath(target->GetGridPosition(), pose.position.ToTf2());

    auto navPath = vector<Vector2>();
    navPath.push_back(pose.position);

    for (auto point : path) {
        navPath.emplace_back(Vector2(point.x(), point.y()));
    }

    _navigatorController->SetPath(navPath);
}
} // namespace Manhattan::Core
