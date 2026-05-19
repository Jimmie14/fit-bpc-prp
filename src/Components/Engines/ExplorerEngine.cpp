#include "Components/ExplorerEngine.hpp"

#include "Components/ArucoDetectionEngine.hpp"
#include "Components/MapThinningUnit.hpp"
#include "Math/Vec3.hpp"
#include "Messages/Nav.hpp"
#include "Viz/Marker.hpp"
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.hpp>

using namespace std;
using namespace Manhattan::nav;

namespace Manhattan::core {
ExplorerEngine::ExplorerEngine(const App& app)
    : RosEngine(app, "explorer")
    , _grid(0, 0, 0)
    , _map(0, 0, 0)
{
    _mapping = app.getComponent<MappingEngine>();

    _app.events->Subscribe<ThinnedMapEvent>([this](const ThinnedMapEvent& event) {
        _grid = event.grid;
        _map = GridMap(_grid.width(), _grid.height(), _grid.resolution());
    });

    _app.events->Subscribe<CodeDetectedEvent>([this](const CodeDetectedEvent& event) {
        OnAruCode(event);
    });
}

void ExplorerEngine::OnEnable()
{
    _state = ExplorerState::Exploring;

    // _mapSubscription = create_subscription<nav_msgs::msg::OccupancyGrid>("slam/grid_thinned", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    //     this->OnMap(msg);
    // });

    _markerPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("explorer/markers", 1);

    _startTimer = create_wall_timer(3s, [this] {
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

ExplorerEngine::ExplorerResult ExplorerEngine::Explore(const Vector3 &inDirection, const Vector2i startCell)
{
    vector<Vector3> path;

    std::set<Vector2i> visited;
    std::optional<Vector2i> frontier = std::nullopt;

    auto dirNorm = inDirection.normalized();
    auto dirs = Vector2i::EightDirections();

    ranges::sort(dirs, [&](const auto& a, const auto& b) {
       const auto va = Vector2(a).toTf2().normalized();
       const auto vb = Vector2(b).toTf2().normalized();

       const auto da = va.dot(dirNorm);
       const auto db = vb.dot(dirNorm);

       return da > db;
   });


    visited.insert(startCell);
    auto next = std::optional(startCell);

    while (next.has_value()) {
        auto current = next.value();

        path.push_back(_map.coordToWorld(current));

        std::vector<Vector2i> options;

        int neighbourCount = 1;
        for (auto dir : dirs) {
            Vector2i neighbour = current + dir;
            if (!_grid.inBounds(neighbour.x, neighbour.y)) continue;

            if (!Walkable(_grid[neighbour])) continue;
            if (visited.contains(neighbour)) continue;

            neighbourCount++;

            options.push_back(neighbour);
            visited.insert(neighbour);
        }

        if (neighbourCount == 2) {
            next = PickFollowingDirection(current, options, dirNorm, dirNorm);
            if (next.has_value())
                dirNorm = (_map.coordToWorld(next.value()) - _map.coordToWorld(current)).normalized();
            continue;
        }

        const auto [ways, newVisited] = GetCrossroadWays({}, current);
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

std::optional<Vector2i> ExplorerEngine::PickFollowingDirection(const Vector2i& current, const vector<Vector2i>& ways, const Vector3& forward, const Vector3& preferred) const
{
    if (ways.empty()) return std::nullopt;

    auto bestScore = std::numeric_limits<double>::max();
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

std::pair<std::vector<Vector2i>, std::set<Vector2i>> ExplorerEngine::GetCrossroadWays(std::set<Vector2i> visited, const Vector2i& start) const
{
    std::vector<Vector2i> ways;
    std::vector<Vector2i> newWays;


    ways.push_back(start);

    int iteration = 0;
    while (!ways.empty()) {
        if (iteration > 4) break;
        iteration++;

        for (auto current : ways) {
            for (auto direction : Vector2i::EightDirections()) {
                const auto next = current + direction;

                if (!Walkable(_grid[next])) continue;
                if (visited.contains(next)) continue;

                newWays.push_back(next);
                visited.insert(next);
            }
        }

        std::swap(ways, newWays);
        newWays.clear();
    }

    std::unordered_set<Vector2i, Vector2iHash> filtered;

    for (const auto& point : ways) {
        bool hasNeighbor = false;

        for (const auto& direction : Vector2i::EightDirections()) {
            if (!filtered.contains(point + direction)) continue;

            hasNeighbor = true;
            break;
        }

        if (hasNeighbor) continue;

        filtered.insert(point);
    }

    vector<Vector2i> result;

    for (auto way : filtered) {
        result.push_back(way);
    }

    return { result, visited };
}

std::optional<Vector2i> ExplorerEngine::ClosestOnThinnedMap(const Vector3& pos) const
{
    const auto intPos = Vector2i(_map.worldToCoord(pos));

    if (Walkable(_grid[intPos]))
        return intPos;

    std::queue<Vector2i> q;
    std::set<Vector2i> visited;

    q.push(intPos);
    visited.insert(intPos);

    while (!q.empty()) {
        auto cell = q.front(); q.pop();

        for (auto direction : Vector2i::EightDirections()) {
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

Vector3 ExplorerEngine::GetPreferredDirection() const
{
    auto getAngleFromId = [](const int id) {
        switch (id) {
        case 0:
        case 10:
            return 0.0;
        case 1:
        case 11:
            return M_PI_2 * 0.5f;
        case 2:
        case 12:
            return -M_PI_2 * 0.5f;
        default:
            return -M_PI * 0.5;
        }
    };

    auto angle = M_PI * 0.5;

    if (_treasureCode.has_value()) {
        // const auto treasureAngle = getAngleFromId(_treasureCode->id);
        //
        // if (_exitCode.has_value()) {
        //     const auto exitAngle = getAngleFromId(_exitCode->id);
        //
        //     angle = exitAngle - treasureAngle;
        // } else {
        //     angle = treasureAngle;
        // }

        angle = getAngleFromId(_treasureCode->id);
    }
    else if (_exitCode.has_value()) {
        angle = getAngleFromId(_exitCode->id);
    }

    return quatRotate(Quaternion(vec3::Up, angle), _junctionEnterDirection);
}

static optional<CodeDetectedEvent> GetCodeFromTressure(const optional<CodeDetectedEvent>& exit, const optional<CodeDetectedEvent>& treasure)
{
    if (!treasure.has_value()) return treasure;
    if (!exit.has_value()) return treasure;

    switch (treasure->id) {
    case 10: // straight
        if (exit->id == 0) return std::nullopt;;
        if (exit->id == 1) return make_optional(CodeDetectedEvent { .id = 2 });
        if (exit->id == 2) return make_optional(CodeDetectedEvent { .id = 1 });
        break;
    case 11: // left
        if (exit->id == 0) return make_optional(CodeDetectedEvent { .id = 2 });
        if (exit->id == 1) return std::nullopt;;
        if (exit->id == 2) return make_optional(CodeDetectedEvent { .id = 0 });
        break;
    case 12: // right
        if (exit->id == 0) return make_optional(CodeDetectedEvent { .id = 2 });
        if (exit->id == 1) return make_optional(CodeDetectedEvent { .id = 0 });
        if (exit->id == 2) return std::nullopt;;
        break;
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

        _currentTarget = ClosestOnThinnedMap(pose.position.toTf2());
        if (!_currentTarget.has_value()) break;


        const auto [ways, visited] = GetCrossroadWays({ }, _currentTarget.value());
        if (ways.size() == 1) {
            if (atDeadEnd == false) {
                atDeadEnd = true;

                // _reverse = !_reverse;
                // _app.Events->Publish(RobotModeChangeEvent { .newMode = RobotMode { .reverse = _reverse } });
            }
        } else {
            atDeadEnd = false;
        }


        if (ways.size() <= 2) {
            _junctionEnterDirection = pose.forward().toTf2();

            if (_inJunction) {
                // if (_treasureCode.has_value()) {
                //     _exitCode = GetCodeFromTressure(_exitCode, _treasureCode);
                //     _treasureCode = std::nullopt;
                // } else {
                //     _exitCode = std::nullopt;
                // }
                _treasureCode = std::nullopt;
                _exitCode = std::nullopt;
            }

            _inJunction = false;
        } else {
            _options.clear();

            for (auto way : ways) {
                _options.push_back(_map.coordToWorld(way));
            }

            const auto preferred = GetPreferredDirection();

            _currentTarget = PickFollowingDirection(_currentTarget.value(), ways, _junctionEnterDirection, preferred);
            _inJunction = true;
        }


        const auto result = Explore(_junctionEnterDirection, _currentTarget.value());

        _currentTarget = result.target;
        _path = result.path;

        _app.events->Publish(RobotFollowPathEvent { .path = _path });
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

    for (auto option : _options) {
        auto marker = viz::marker::point(option, "map");

        marker.color = viz::marker::color(1, 0.5, 0);

        markers.add(marker);
    }

    _markerPublisher->publish(markers.array);
}

void ExplorerEngine::OnAruCode(CodeDetectedEvent aruCode)
{
    std::lock_guard lock(_mutex);

    if (aruCode.id >= 10) _treasureCode = aruCode;
    else _exitCode = aruCode;

     std::cout << "Aruco code detected with id: " << aruCode.id << " with direction: " << aruCode.pose.forward().x << " " << aruCode.pose.forward().y << std::endl;
}

} // namespace Manhattan::Core
