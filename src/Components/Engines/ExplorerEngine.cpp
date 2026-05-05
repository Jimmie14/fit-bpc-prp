#include "ExplorerEngine.hpp"

#include "MapThinningUnit.hpp"
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

    _app.Events->Subscribe<ThinnedMapEvent>([this](const ThinnedMapEvent& event) {
        _grid = event.grid;
        _map = GridMap(_grid.width(), _grid.height(), _grid.resolution());
    });
}

void ExplorerEngine::OnEnable()
{
    _state = ExplorerState::Exploring;

    // _mapSubscription = create_subscription<nav_msgs::msg::OccupancyGrid>("slam/grid_thinned", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    //     this->OnMap(msg);
    // });

    _markerPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("explorer/markers", 1);

    _startTimer = create_wall_timer(1s, [this] {
        _timer = create_wall_timer(100ms, [this] { Update(); });
        _startTimer->reset();
    });

    _publishTimer = create_wall_timer(1s, [this] {
       Publish();
    });

}

void ExplorerEngine::OnDisable()
{
    _mapSubscription.reset();
    _markerPublisher.reset();

    _timer.reset();
    _publishTimer.reset();
}

static bool Walkable(const bool value)
{
    return value;
}

ExplorerEngine::ExplorerResult ExplorerEngine::Explore(const Vector3 &inDirection, const Vector2Int startCell) const
{
    std::unordered_map<Vector2Int, Vector2Int, Vector2IntHash> previous;
    std::queue<Vector2Int> queue;
    std::set<Vector2Int> visited;

    queue.push(startCell);
    visited.insert(startCell);
    std::optional<Vector2Int> frontier = std::nullopt;

    bool beforeJunction = true;

    const auto dirNorm = inDirection.normalized();
    auto dirs = Vector2Int::EightDirections();

    // Sort directions to prioritize inDirection
    ranges::sort(dirs, [&](const auto& a, const auto& b) {
        const auto va = Vector2(a).ToTf2().normalized();
        const auto vb = Vector2(b).ToTf2().normalized();

        const auto da = va.dot(dirNorm);
        const auto db = vb.dot(dirNorm);

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

            if (!Walkable(_grid[neighbor])) continue;

            neighbourCount++;

            if (visited.contains(neighbor)) continue;

            previous[neighbor] = current;
            queue.push(neighbor);
            visited.insert(neighbor);
        }

        if (neighbourCount == 2) continue;

        const auto [ways, newVisited] = GetCrossroadWays(current, dirs);
        if (ways.size() == 2 || ways.empty()) continue;

        if (beforeJunction) {
            beforeJunction = false;

            auto max = -1.0;
            auto newWay = ways.front();

            for (auto point : ways) {
                const auto dir = (_map.coordToWorld(point) - _map.coordToWorld(current)).normalized();


                const auto dot = dir.dot(dirNorm);
                if (dot > max) {
                    max = dot;
                    newWay = point;
                }
            }

            queue = std::queue<Vector2Int>();
            queue.push(newWay);

            previous[newWay] = current;
            visited = newVisited;

            continue;
        }


        frontier = current;
        break;
    }

    const auto target = frontier;

    std::vector<Vector3> path;
    if (!frontier.has_value())
        return ExplorerResult{ target, path };

    Vector2Int current = *frontier;

    while (current != startCell) {
        path.push_back(_map.coordToWorld(current));

        const auto it = previous.find(current);
        if (it == previous.end())
            return ExplorerResult{ target, {} };

        current = it->second;
    }

    ranges::reverse(path);
    return ExplorerResult{ target, path };
}

std::pair<std::vector<Vector2Int>, std::set<Vector2Int>> ExplorerEngine::GetCrossroadWays(const Vector2Int& start, vector<Vector2Int>& directions) const
{
    std::vector<Vector2Int> ways;
    std::vector<Vector2Int> newWays;
    std::set<Vector2Int> visited;

    ways.push_back(start);

    int iteration = 0;
    while (!ways.empty()) {
        if (iteration > 4) break;
        iteration++;

        for (auto current : ways) {
            for (auto direction : directions) {
                const auto next = current + direction;

                if (visited.contains(next)) continue;
                if (!Walkable(_grid[next])) continue;

                newWays.push_back(next);
                visited.insert(next);
            }
        }

        std::swap(ways, newWays);
        newWays.clear();


    }

    return { ways, visited };
}

std::optional<Vector2Int> ExplorerEngine::ClosestOnThinnedMap(const Vector3& pos) const
{
    const auto intPos = Vector2Int(_map.worldToCoord(pos));

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

        // if (!_currentTarget.has_value()) {
        //     _currentTarget = ClosestOnThinnedMap(pose.position);
        //
        //     if (!_currentTarget.has_value())
        //         return;
        //
        //     auto target = *_currentTarget;
        //     std::cout << "Got new target: " << target.x << " " << target.y << std::endl;
        // }

        _currentTarget = ClosestOnThinnedMap(pose.position.ToTf2());
        if (!_currentTarget.has_value()) return;


        const auto result = Explore(pose.forward.ToTf2(), _currentTarget.value());


        _currentTarget = result.target;
        _path = result.path;

        auto navPath = vector<Vector2>();

        for (auto point : _path) {
            navPath.emplace_back(Vector2(point.x(), point.y()));
        }

        _navigatorController->SetPath(navPath);
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
        const auto worldPoint = _map.coordToWorld(_currentTarget.value());

        markers.add(viz::marker::point(worldPoint, "map"));
    }

    for (auto path : _path) {
        markers.add(viz::marker::point(path, "map"));
    }

    _markerPublisher->publish(markers.array);
}

} // namespace Manhattan::Core
