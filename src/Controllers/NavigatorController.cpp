#include "NavigatorController.hpp"

#include "OdometryEngine.hpp"
#include "SplinePath.hpp"

using namespace std;

constexpr int rayCount = 32;
constexpr double rayDistance = 3;
constexpr double avoidanceDistance = 0.22;
constexpr double avoidanceStrength = 20.0;

constexpr double aimDistance = 0.2;
constexpr double destinationDistance = 0.3;
constexpr double distanceToSlow = 0.5;

constexpr int lookAheadWaypoints = 3;
constexpr double cornerSlowMinFactor = 0.25;
constexpr double cornerSlowAngleThreshold = M_PI / 6.0;
constexpr double cornerSlowAngleMax = M_PI / 2.0;

constexpr double maxLinearSpeed = 0.2;
constexpr double maxAngularSpeed = 0.2;

constexpr double turnDeceleration = 1.5;
constexpr double acceleration = 0.05;
constexpr double deceleration = 0.4;

constexpr double angularKp = 0.2;
constexpr double angularKi = 0.012;
constexpr double angularKd = 0.001;

static double MoveTowards(const double current, const double target, const double maxDelta)
{
    const auto delta = target - current;
    if (std::abs(delta) <= maxDelta)
        return target;

    return current + (delta > 0 ? maxDelta : -maxDelta);
}

namespace Manhattan::Core {
NavigatorController::NavigatorController(const App& app)
    : RosConnector(app)
    , _kinematics(app.GetController<OdometryEngine>()->GetKinematics())
    , _angularPid(angularKp, angularKi, angularKd)
    , _lastTime(std::chrono::steady_clock::now())
{

    _motor = app.GetController<MotorDriver>();
    _slam = app.GetController<MappingEngine>();

    _pathPublisher = _node->create_publisher<nav_msgs::msg::Path>("~/nav/desired_path", 10);
    _rayCastPublisher = _node->create_publisher<visualization_msgs::msg::MarkerArray>("~/nav/ray_cast", 10);

    _timer = _node->create_wall_timer(10ms, // todo: timer frequency config duplication
        [this] { Update(); });
}

void NavigatorController::SetPath(const std::vector<GridCell*>& path)
{
    const auto waypoints = SmoothPath(path);
    // for (const auto waypoint : path)
    //     waypoints.push_back(waypoint->GetWorldPosition());

    _path.Initialize(waypoints);
}

void NavigatorController::PublishPath() const
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = _node->now();

    auto poseMsg = geometry_msgs::msg::PoseStamped();
    poseMsg.header.stamp = msg.header.stamp;
    poseMsg.header.frame_id = "map";

    for (const auto& seg : _path.GetSegments()) {
        for (int i = 1; i <= 60; i++) {
            const auto position = seg.Evaluate(i / 60.0);

            poseMsg.pose.position.x = position.x;
            poseMsg.pose.position.y = position.y;
            poseMsg.pose.position.z = 0.0;

            msg.poses.push_back(poseMsg);
        }
    }

    _pathPublisher->publish(msg);
}

void NavigatorController::SetDestination(GridCell* destination)
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

    const auto startCell = _slam->GetCell(_slam->CurrentPose().position);
    if (startCell == nullptr || destination == nullptr)
        return SetPath({});

    if (startCell == destination)
        return SetPath({});

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
        return SetPath({});

    std::vector<GridCell*> path;
    auto current = destination;

    while (current != nullptr && current != startCell) {
        path.push_back(current);

        const auto it = previous.find(current);
        if (it == previous.end()) {
            return SetPath({});
        }

        current = it->second;
    }

    ranges::reverse(path);
    // path = SmoothPath(path);

    // std::queue<GridCell*> result;
    // for (auto* cell : path) {
    //     result.push(cell);
    // }

    SetPath(path);
}


std::vector<Vector2> NavigatorController::SmoothPath(const std::vector<GridCell*>& path) const
{
    std::vector<Vector2> points;
    points.reserve(path.size());
    for (const auto* cell : path) {
        if (cell) {
            points.push_back(cell->GetWorldPosition());
        }
    }

    if (points.size() <= 2) {
        return points;
    }

    size_t currentIndex = 0;

    while (currentIndex < points.size() - 1) {
        size_t furthestIndex = currentIndex + 1;

        // Look for the furthest point we can see without hitting an obstacle
        for (size_t i = points.size() - 1; i > currentIndex + 1; i--) {
            const Vector2& p1 = points[currentIndex];
            const Vector2& p2 = points[i];

            Vector2 dir = (p2 - p1).Normalized();
            double dist = Vector2::Distance(p1, p2);

            RayHit hitInfo;

            if (!_slam->RayCast(p1, dir, hitInfo, dist)) {
                furthestIndex = i;
                break;
            }
        }

        const Vector2& startPt = points[currentIndex];
        const Vector2& endPt = points[furthestIndex];
        Vector2 lineDir = (endPt - startPt).Normalized();

        // Project all intermediate staircase points onto the straight line segment
        // to smooth out the transition between the two points
        for (size_t j = currentIndex + 1; j < furthestIndex; j++) {
            Vector2 v = points[j] - startPt;
            double d = Vector2::Dot(v, lineDir);
            points[j] = startPt + lineDir * d;
        }

        currentIndex = furthestIndex;
    }

    return points;
}

void NavigatorController::ClearPath()
{
    _path.Initialize({});
    _t = 0.0;
}

bool NavigatorController::IsInDestination() const
{
    const auto pose = _slam->CurrentPose();
    const auto result = _path.FindClosestPoint(pose.position);

    return _path.GetTotalLength() - result.DistanceAlongPath <= destinationDistance || !_path.HasPath();
    // return _t > 0.9 || !_path.HasPath();
}

vector<RayHit> NavigatorController::RayCastAround(const Pose& pose) const
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

void NavigatorController::PublishRayCast(const vector<RayHit>& hits, const Pose& pose,
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
        marker.header.stamp = _node->now();
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
    marker.header.stamp = _node->now();
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

Vector2 NavigatorController::GetDirection(const vector<RayHit>& rayHits, const Pose& pose,
    const Vector2& desiredDirection) const
{
    auto direction = desiredDirection;
    constexpr auto rayWeight = 1.0 / rayCount;

    for (const auto rayHit : rayHits) {
        const auto dst = Vector2::Distance(pose.position, rayHit.hit);

        if (dst >= avoidanceDistance)
            continue;

        const auto proximity = 1.0 - dst / avoidanceDistance;
        const auto pushForce = proximity * proximity * avoidanceStrength;

        direction = direction + rayHit.normal * (pushForce * rayWeight);
    }

    return direction.Normalized();
}

double NavigatorController::GetCornerSlowFactor(const Pose& pose, const double currentT) const
{
    if (!_path.HasPath())
        return 1.0;

    std::vector<Vector2> points;
    points.reserve(lookAheadWaypoints + 1);
    points.push_back(pose.position);

    auto t = currentT;
    for (int i = 0; i < lookAheadWaypoints && t < 1.0; ++i) {
        points.push_back(_path.GetPointAtDistance(t * _path.GetTotalLength()));
        t = std::clamp(t + aimDistance, 0.0, 1.0);
    }

    if (points.size() < 3)
        return 1.0;

    double worstAngle = 0.0;

    for (size_t i = 0; i + 2 < points.size(); ++i) {
        const auto a = (points[i + 1] - points[i]).Normalized();
        const auto b = (points[i + 2] - points[i + 1]).Normalized();
        const auto angle = std::abs(Vector2::SignedAngle(a, b));
        worstAngle = std::max(worstAngle, angle);
    }

    if (worstAngle <= cornerSlowAngleThreshold)
        return 1.0;

    t = clamp((worstAngle - cornerSlowAngleThreshold) / (cornerSlowAngleMax - cornerSlowAngleThreshold), 0.0, 1.0);
    return 1.0 - t * (1.0 - cornerSlowMinFactor);
}

void NavigatorController::Update()
{
    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> delta = now - _lastTime;
    auto deltaTime = delta.count();

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

    _t = std::clamp((result.DistanceAlongPath + aimDistance) / _path.GetTotalLength(), 0.0, 1.0);

    auto aimPoint = _path.GetPointAtDistance(_t * _path.GetTotalLength());
    auto directionToWaypoint = (aimPoint - pose.position).Normalized();

    // auto currentWaypoint = _path.front();
    // auto directionToWaypoint = (currentWaypoint->GetWorldPosition() - pose.position).Normalized();
    //
    // if (Vector2::Distance(pose.position, currentWaypoint->GetWorldPosition()) < waypointTolerance) {
    //     _path.pop();
    //     return;
    // }

    const auto rayHits = RayCastAround(pose);
    const auto desiredDirection = GetDirection(rayHits, pose, directionToWaypoint);

    PublishRayCast(rayHits, pose, desiredDirection);

    const auto angleToTarget = Vector2::SignedAngle(pose.forward, desiredDirection);
    // const auto angleToTarget = Vector2::SignedAngle(pose.forward, directionToWaypoint);

    const auto angularSpeedTarget = clamp(_angularPid.step(angleToTarget, deltaTime), -maxAngularSpeed, maxAngularSpeed);
    _currentAngularVelocity = angularSpeedTarget;

    auto distanceFactor = 1.0;
    if (!rayHits.empty()) {
        const auto forwardHit = rayHits[0];
        const auto distanceAhead = Vector2::Distance(forwardHit.hit, pose.position);

        distanceFactor = clamp(distanceAhead / distanceToSlow, 0.0, 1.0);
    }

    const auto turnFactor = clamp(exp(-turnDeceleration * abs(angleToTarget)), 0.0, 1.0);
    const auto cornerFactor = GetCornerSlowFactor(pose, _t);

    const auto targetSpeed = maxLinearSpeed * clamp(distanceFactor * cornerFactor * turnFactor, 0.0, 1.0);

    _currentLinearVelocity = MoveTowards(_currentLinearVelocity, targetSpeed,
        (targetSpeed > _currentLinearVelocity ? acceleration : deceleration) * maxLinearSpeed * deltaTime);

    const auto speed = _kinematics.inverse(RobotSpeed { _currentLinearVelocity, _currentAngularVelocity });
    _motor->SetForce(speed.left, speed.right);
}

} // namespace Manhattan::Core
