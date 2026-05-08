#include "Components/MazeEngine.hpp"

#include "Viz/Grid.hpp"
#include "Viz/Marker.hpp"

using namespace std;

namespace Manhattan::core {

using namespace Manhattan::nav;

MazeEngine::MazeEngine(const App& app)
    : RosEngine(app, "maze"), _thinned_map(0,0,0)
{
    _navigator = app.getComponent<NavigatorEngine>();
    _mapping = app.getComponent<MappingEngine>();

    _graphPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("maze/graph", 10);
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
    const auto pose = Pose::fromRosPoseMessage(msg->pose);
}

void MazeEngine::OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
    _thinned_map = viz::nav::ToOccupancyGrid(*msg, 50);
}

void MazeEngine::PublishCurrenThGraph() const
{
    if (!_currentWayPoint)
        return;

    visualization_msgs::msg::MarkerArray markerArray;

    // Track visited waypoints to avoid cycles
    std::set<std::shared_ptr<WayPoint>> visitedWaypoints;
    std::queue<std::shared_ptr<WayPoint>> queue;

    queue.push(_currentWayPoint);

    int id = 0;
    while (!queue.empty()) {
        auto wp = queue.front();
        queue.pop();

        if (visitedWaypoints.contains(wp)) continue;
        visitedWaypoints.insert(wp);

        // 1. Create Sphere for WayPoint
        visualization_msgs::msg::Marker waypointMarker;
        waypointMarker.header.frame_id = "map"; // Adjust to your world frame
        waypointMarker.header.stamp = now();
        waypointMarker.ns = "waypoints";
        waypointMarker.id = id++;
        waypointMarker.type = visualization_msgs::msg::Marker::SPHERE;
        waypointMarker.action = visualization_msgs::msg::Marker::ADD;
        waypointMarker.pose.position.x = wp->position.x;
        waypointMarker.pose.position.y = wp->position.y;
        waypointMarker.pose.position.z = 0.1;
        waypointMarker.scale.x = 0.2;
        waypointMarker.scale.y = 0.2;
        waypointMarker.scale.z = 0.2;
        waypointMarker.color.a = 1.0;
        waypointMarker.color.r = 1.0; // Red for waypoints
        waypointMarker.color.g = 0.0;
        waypointMarker.color.b = 0.0;
        markerArray.markers.push_back(waypointMarker);

        // 2. Create Line Strips for Connections
        for (const auto& connection : wp->connected) {
            if (!connection.target) continue;

            visualization_msgs::msg::Marker pathMarker;
            pathMarker.header.frame_id = "map";
            pathMarker.header.stamp = now();
            pathMarker.ns = "connections";
            pathMarker.id = id++;
            pathMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            pathMarker.action = visualization_msgs::msg::Marker::ADD;
            pathMarker.scale.x = 0.05; // Line width
            pathMarker.color.a = 0.8;
            pathMarker.color.r = 0.0;
            pathMarker.color.g = 1.0; // Green for connections
            pathMarker.color.b = 0.0;

            // Add start point
            geometry_msgs::msg::Point pStart;
            pStart.x = wp->position.x;
            pStart.y = wp->position.y;
            pathMarker.points.push_back(pStart);

            // Add path points
            for (const auto& step : connection.path) {
                geometry_msgs::msg::Point p;
                p.x = step.x;
                p.y = step.y;
                pathMarker.points.push_back(p);
            }

            // Add end point (target position)
            geometry_msgs::msg::Point pEnd;
            pEnd.x = connection.target->position.x;
            pEnd.y = connection.target->position.y;
            pathMarker.points.push_back(pEnd);

            markerArray.markers.push_back(pathMarker);

            // Add neighbor to queue to continue traversal
            if (!visitedWaypoints.contains(connection.target)) {
                queue.push(connection.target);
            }
        }
    }

    _graphPublisher->publish(markerArray);
}

void MazeEngine::Update() {
    if (_thinned_map.resolution() == 0 || !_navigator->IsInDestination())
        return;

    Init();
    PublishCurrenThGraph();

    std::cout << _currentWayPoint->connected.size() << std::endl;
    return;

    const auto target = NextJunction(_currentWayPoint);
    if (target == nullptr)
        return;

    _currentWayPoint->visited = true;
    _currentWayPoint = target;

    const auto cell = _mapping->GetCell(target->position);
    _navigator->SetDestination(cell);
}

std::vector<Vector2Int> MazeEngine::GetValidNeighbors(const Vector2Int& cell) {
    std::vector<Vector2Int> result;

    for (auto dir : Vector2Int::EightDirections()) {
        Vector2Int next = cell + dir;

        if (!_thinned_map[next]) continue;
        result.push_back(next);
    }

    return result;
}

bool MazeEngine::IsWaypoint(const Vector2Int& cell) {
    auto neighbors = GetValidNeighbors(cell);
    return neighbors.size() != 2;
}

std::shared_ptr<MazeEngine::WayPoint> MazeEngine::WalkUntilWaypoint(Vector2Int prev, Vector2Int current) {
    auto waypoint = std::make_shared<WayPoint>();
    waypoint->connected = {};

    std::vector<Vector2> path;
    std::set<Vector2Int> visited;

    while (true) {
        auto neighbors = GetValidNeighbors(current);

        std::erase(neighbors, prev);

        if (IsWaypoint(current)) {
            waypoint->position = _mapping->GridToWorld(current);
            waypoint->connected.push_back({ .target = nullptr, .path = path });
            break;
        }

        visited.insert(current);
        path.emplace_back(_mapping->GridToWorld(current));

        prev = current;
        current = neighbors[0];
    }

    if (path.size() < 5) return nullptr;
    return waypoint;
}

std::shared_ptr<MazeEngine::WayPoint> MazeEngine::Init()
{
    const auto initPos = ClosestOnThinnedMap(_mapping->CurrentPose().position);
    if (!initPos) return nullptr;

    _currentWayPoint = std::make_shared<WayPoint>();

    for (auto dir : Vector2Int::EightDirections()) {
        auto wp = WalkUntilWaypoint(initPos.value(), initPos.value() + dir);
        if (!wp) continue;

        auto connection = WayPoint::Connection();
        connection.path = wp->connected[0].path;
        ranges::reverse(connection.path);

        connection.target = wp;
        wp->connected[0].target = _currentWayPoint;

        _currentWayPoint->connected.push_back(connection);
    }

    return _currentWayPoint;
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

            if (neighbour.x >= _thinned_map.width() || neighbour.x < 0 || neighbour.y >= _thinned_map.height() || neighbour.y < 0)
                continue;

            if (_thinned_map[neighbour])
                return neighbour;

            q.push(neighbour);
        }

        visited.insert(cell);
    }

    return std::nullopt;
}

std::shared_ptr<MazeEngine::WayPoint> MazeEngine::NextJunction(const std::shared_ptr<WayPoint>& current) {
    if (!current) return Init();

    const auto initPos = ClosestOnThinnedMap(current->position);
    if (!initPos) return nullptr;

    std::queue<Vector2Int> q;
    std::set<Vector2Int> visited;

    q.push(initPos.value());
    visited.insert(q.front());

    while (!q.empty()) {
        auto cell = q.front(); q.pop();
        int traversable_neighbors = 0;
        std::vector<Vector2Int> neighbors;
    }

    return nullptr;
}

} // namespace Manhattan::Core