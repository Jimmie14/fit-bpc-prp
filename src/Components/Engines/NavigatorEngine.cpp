#include "NavigatorEngine.hpp"

#include "OdometryEngine.hpp"

using namespace std;

constexpr int rayCount = 32;
constexpr double rayDistance = 1;
constexpr double avoidanceDistance = 0.2;
constexpr double avoidanceStrength = 2.0;

constexpr double aimDistance = 0.3;
constexpr double destinationDistance = 0.1;
constexpr double distanceToSlow = 0.5;

constexpr int lookAheadWaypoints = 3;
constexpr double cornerSlowMinFactor = 0.25;
constexpr double cornerSlowAngleThreshold = M_PI / 6.0;
constexpr double cornerSlowAngleMax = M_PI / 2.0;

constexpr double maxLinearSpeed = 0.15;
constexpr double maxAngularSpeed = 0.15;

constexpr double turnDeceleration = 1.5;
constexpr double acceleration = 0.05;
constexpr double deceleration = 0.4;

constexpr double angularKp = 0.4;
constexpr double angularKi = 0.0;
constexpr double angularKd = 0.1;

static double MoveTowards(const double current, const double target, const double maxDelta)
{
    const auto delta = target - current;
    if (std::abs(delta) <= maxDelta)
        return target;

    return current + (delta > 0 ? maxDelta : -maxDelta);
}

namespace Manhattan::Core {
NavigatorEngine::NavigatorEngine(const App& app)
    : RosEngine(app, "navigator")
    , _kinematics(app.GetComponent<OdometryEngine>()->GetKinematics())
    , _angularPid(angularKp, angularKi, angularKd)
    , _lastTime(std::chrono::steady_clock::now())
{
    _motor = app.GetComponent<MotorDriver>();
    _slam = app.GetComponent<MappingEngine>();

    _pathPublisher = create_publisher<nav_msgs::msg::Path>("nav/desired_path", 1);
    _rayCastPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("nav/ray_cast", 1);

    _timer = create_wall_timer(10ms,
        [this] { Update(); });

    _app.Events->Subscribe<MappingEngineStateChangeEvent>([this](const MappingEngineStateChangeEvent& event) {
        this->OnMappingEngineStateChange(event);
    });
}

void NavigatorEngine::SetPath(const std::vector<Vector2>& path)
{
    _path.Initialize(path);
}

void NavigatorEngine::PublishPath() const
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();

    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header = msg.header;

    if (!_path.HasPath())
        return;

    constexpr int samples = 500;
    const double totalLength = _path.GetTotalLength();

    for (int i = 0; i <= samples; i++) {
        double distance = (static_cast<double>(i) / samples) * totalLength;
        const auto position = _path.GetPointAtDistance(distance);

        poseMsg.pose.position.x = position.x;
        poseMsg.pose.position.y = position.y;
        poseMsg.pose.position.z = 0.0;

        msg.poses.push_back(poseMsg);
    }

    _pathPublisher->publish(msg);
}

void NavigatorEngine::SetDestination(GridCell* destination)
{
    struct QueueItem {
        GridCell* cell;
        double distance;

        bool operator>(const QueueItem& other) const
        {
            return distance > other.distance;
        }
    };

    std::map<GridCell*, double> distances;
    std::map<GridCell*, GridCell*> previous;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<>> openSet;
    std::unordered_set<GridCell*> visited;

    std::vector<Vector2> points;

    const auto startCell = _slam->GetCell(_slam->CurrentPose().position);
    if (startCell == nullptr || destination == nullptr)
        return SetPath(points);

    if (startCell == destination)
        return SetPath(points);

    distances[startCell] = 0.0;
    openSet.push({ startCell, 0.0 });

    while (!openSet.empty()) {
        const auto current = openSet.top().cell;
        openSet.pop();

        if (current == nullptr)
            continue;
        if (visited.contains(current))
            continue;
        visited.insert(current);

        if (current == destination)
            break;

        for (GridCell* neighbor : _slam->GetNeighbors(current)) {
            if (neighbor == nullptr)
                continue;
            if (neighbor->IsOccupied())
                continue;

            const auto alt = distances[current] + neighbor->GetCost();

            const auto distIterator = distances.find(neighbor);
            if (distIterator != distances.end() && alt >= distIterator->second)
                continue;

            distances[neighbor] = alt;
            previous[neighbor] = current;
            openSet.push({ neighbor, alt });
        }
    }

    if (!distances.contains(destination))
        return SetPath(points);

    std::vector<GridCell*> path;
    auto current = destination;

    while (current != nullptr && current != startCell) {
        path.push_back(current);

        const auto it = previous.find(current);
        if (it == previous.end()) {
            return SetPath(points);
        }

        current = it->second;
    }

    ranges::reverse(path);

    points.reserve(path.size());
    for (const auto* cell : path) {
        if (cell)
            points.push_back(cell->GetWorldPosition());
    }

    SetPath(points);
}


void NavigatorEngine::ClearPath()
{
    _path.Initialize({});
}

bool NavigatorEngine::IsInDestination() const
{
    const auto pose = _slam->CurrentPose();
    const auto result = _path.FindClosestPoint(pose.position);

    return _path.GetTotalLength() - result.DistanceAlongPath <= destinationDistance || !_path.HasPath();
    // return _t > 0.9 || !_path.HasPath();
}

vector<RayHit> NavigatorEngine::RayCastAround(const Pose& pose) const
{
    vector<RayHit> hits;

    auto angle = pose.rotation + M_PI * 0.5f;
    const auto angleStep = M_PI * 2 / rayCount;

    for (auto i = 0; i < rayCount; i++) {
        RayHit rayHit;
        const auto hit = _slam->RayCast(pose.position, Vector2(cos(angle), sin(angle)), rayHit, rayDistance);

        angle += angleStep;
        if (!hit)
            continue;

        hits.push_back(rayHit);
    }

    return hits;
}

void NavigatorEngine::PublishRayCast(const vector<RayHit>& hits, const Pose& pose,
    const Vector2& desiredDirection) const
{
    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker clearMarker;
    clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(clearMarker);

    int id = 0;
    for (const auto& rayHit : hits) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "raycasts";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01; // Line width

        geometry_msgs::msg::Point start, end;
        start.x = pose.position.x;
        start.y = pose.position.y;
        start.z = 0.1;
        end.x = rayHit.hit.x;
        end.y = rayHit.hit.y;
        end.z = 0.1;

        marker.points.push_back(start);
        marker.points.push_back(end);

        auto isClose = Vector2::Distance(pose.position, rayHit.hit) < avoidanceDistance;

        marker.color.r = isClose ? 1.0 : 0.0;
        marker.color.g = 0.0;
        marker.color.b = isClose ? 0.0 : 1.0;
        marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "raycasts";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01; // Line width

    geometry_msgs::msg::Point start, end;
    start.x = pose.position.x;
    start.y = pose.position.y;
    start.z = 0.1;
    end.x = pose.position.x + desiredDirection.x;
    end.y = pose.position.y + desiredDirection.y;
    end.z = 0.1;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    markerArray.markers.push_back(marker);

    _rayCastPublisher->publish(markerArray);
}

Vector2 NavigatorEngine::GetDirection(const vector<RayHit>& rayHits, const Pose& pose,
    const Vector2& desiredDirection)
{
    Vector2 avoidance(0, 0);
    for (const auto& rayHit : rayHits) {
        const auto dist = Vector2::Distance(pose.position, rayHit.hit);
        if (dist >= avoidanceDistance) continue;

        const auto proximity = 1.0 - dist / avoidanceDistance;
        const auto force = std::pow(proximity, 2) * avoidanceStrength;
        avoidance = avoidance + rayHit.normal * force;
    }

    return (desiredDirection + avoidance).Normalized();
}

double NavigatorEngine::GetLinearVelocity(const Pose& pose, const double t, const double delta) const
{
    const auto d = Vector2::Dot(pose.forward, (_path.GetPointAtDistance(t * _path.GetTotalLength()) - pose.position).Normalized());
    const auto difference = d * d * d;

    const auto targetSpeed = maxLinearSpeed * clamp(difference, 0.0, 1.0) * difference;
    const auto acc = targetSpeed > _currentLinearVelocity ? acceleration : deceleration;

    return MoveTowards(_currentLinearVelocity, targetSpeed, acc * maxLinearSpeed * delta);
}

void NavigatorEngine::Update()
{
    const auto now = std::chrono::steady_clock::now();
    const duration<double> delta = now - _lastTime;
    const auto deltaTime = delta.count();

    _lastTime = now;

    if (!_path.HasPath()) {
        const auto speed = _kinematics.inverse(RobotSpeed { 0, 0 });

        _motor->SetForce(speed.left, speed.right);
        return;
    }

    PublishPath();

    auto pose = _slam->CurrentPose();
    auto result = _path.FindClosestPoint(pose.position);

    if (_path.GetTotalLength() <= 0)
        return;

    const auto t = std::clamp((result.DistanceAlongPath + aimDistance) / _path.GetTotalLength(), 0.0, 1.0);

    const auto aimPoint = _path.GetPointAtDistance(t * _path.GetTotalLength());
    const auto rayHits = RayCastAround(pose);
    const auto desiredDirection = GetDirection(rayHits, pose, (aimPoint - pose.position).Normalized());

    PublishRayCast(rayHits, pose, desiredDirection);

    const auto speedRatio = abs(_currentLinearVelocity) / maxLinearSpeed;
    const float turnFactor = pow(1.0 - speedRatio, 2);

    const auto maxTurnAtSpeed = maxAngularSpeed * turnFactor;
    const auto angleToTarget = Vector2::SignedAngle(pose.forward, desiredDirection);

    _currentAngularVelocity = clamp(_angularPid.step(angleToTarget, deltaTime), -maxTurnAtSpeed, maxTurnAtSpeed);
    _currentLinearVelocity = GetLinearVelocity(pose, t, deltaTime);

    const auto speed = _kinematics.inverse(RobotSpeed { _currentLinearVelocity, _currentAngularVelocity });
    _motor->SetForce(speed.left, speed.right);
}

void NavigatorEngine::OnMappingEngineStateChange(MappingEngineStateChangeEvent event)
{
    if (event.newState == MappingEngineState::Lost) {
        ClearPath();
    }

    if (event.oldState == MappingEngineState::Lost && event.newState == MappingEngineState::Stable) {
        ClearPath();
    }
}

} // namespace Manhattan::Core
