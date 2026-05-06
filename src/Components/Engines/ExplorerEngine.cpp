#include "ExplorerEngine.hpp"

#include "MapThinningUnit.hpp"
#include "Math/Vec3.hpp"
#include "Viz/Marker.hpp"
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.hpp>

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
    vector<Vector3> path;

    std::set<Vector2Int> visited;
    std::optional<Vector2Int> frontier = std::nullopt;

    auto dirNorm = inDirection.normalized();
    auto dirs = Vector2Int::EightDirections();


    visited.insert(startCell);
    auto next = std::optional(startCell);

    const auto junction= GetCrossroadWays(startCell, dirs);
    if (junction.first.size() > 1) {
        const auto preferred = quatRotate(Quaternion(vec3::Up, -M_PI * 0.5), dirNorm);
        const auto result = PickFollowingDirection(startCell, junction.first, dirNorm, preferred);

        visited = junction.second;
        next = result;

        std::cout << "Before juction with count: " << junction.first.size() << std::endl;
        std::cout << "Picking direction " << next.value().toString() << std::endl;
    }

    while (next.has_value()) {
        auto current = next.value();
        path.push_back(_map.coordToWorld(current));

        std::vector<Vector2Int> options;

        int neighbourCount = 0;
        for (auto dir : dirs) {
            Vector2Int neighbour = current + dir;
            if (!_grid.inBounds(neighbour.x, neighbour.y)) continue;

            if (!Walkable(_grid[neighbour])) continue;

            neighbourCount++;

            if (visited.contains(neighbour)) continue;

            options.push_back(neighbour);
            visited.insert(neighbour);
        }

        if (neighbourCount == 0) {
            next = std::nullopt;
            continue;
        }

        if (neighbourCount == 2) {
            next = PickFollowingDirection(current, options, dirNorm, dirNorm);
            if (next.has_value())
                dirNorm = (_map.coordToWorld(next.value()) - _map.coordToWorld(current)).normalized();
            continue;
        }

        const auto [ways, newVisited] = GetCrossroadWays(current, dirs);
        if (ways.size() <= 2) {
            next = PickFollowingDirection(current, options, dirNorm, dirNorm);
            if (next.has_value())
                dirNorm = (_map.coordToWorld(next.value()) - _map.coordToWorld(current)).normalized();
            continue;
        }

        frontier = current;
        break;
    }

    const auto target = frontier;

    return ExplorerResult{ target, path };
}

std::optional<Vector2Int> ExplorerEngine::PickFollowingDirection(const Vector2Int& current, const vector<Vector2Int>& ways, const Vector3& forward, const Vector3& preferred) const
{
    if (ways.empty()) return std::nullopt;

    auto bestScore = std::numeric_limits<float>::max();
    auto best = ways.front();

    for (auto point : ways) {
        const auto dir = (_map.coordToWorld(point) - _map.coordToWorld(current)).normalized();
        if (forward.dot(dir) < -0.4f) continue;

        const auto forwardAngle = forward.angle(dir);
        const auto preferredAngle = preferred.angle(dir);

        const auto score = preferredAngle + 0.25f * forwardAngle;
        if (score >= bestScore) continue;

        bestScore = score;
        best = point;
    }

    return best;
}

std::pair<std::vector<Vector2Int>, std::set<Vector2Int>> ExplorerEngine::GetCrossroadWays(const Vector2Int& start, const vector<Vector2Int>& directions) const
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
    if (_grid.size() == 0) return;

    switch (_state) {
    case ExplorerState::Idle:
        break;

    case ExplorerState::Exploring: {
        const auto pose = _mapping->CurrentPose();

        _currentTarget = ClosestOnThinnedMap(pose.position.ToTf2());
        if (!_currentTarget.has_value()) break;

        // const auto [ways, _] = GetCrossroadWays(_currentTarget.value(), Vector2Int::EightDirections());
        // if (ways.size() > 2 && !_navigatorController->IsInDestination()) break;

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
