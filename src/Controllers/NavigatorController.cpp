#include "NavigatorController.hpp"

#include "RobotOdometry.hpp"

using namespace std;

constexpr int rayCount = 16;
constexpr double rayDistance = 2;
constexpr double avoidanceDistance = 0.3;
constexpr double avoidanceStrength = 20;

constexpr double waypointTolerance = 0.2;

constexpr double maxLinearSpeed = 0.2;
constexpr double maxAngularSpeed = 0.2;

constexpr double turnDeceleration = 8.0;
constexpr double acceleration = 0.05;
constexpr double deceleration = 0.4;
constexpr double updateDeltaTime = 0.01;

constexpr double angularKp = 0.2;
constexpr double angularKi = 0.01;
constexpr double angularKd = 0.0;

static double MoveTowards(const double current, const double target, const double maxDelta) {
    const auto delta = target - current;
    if (std::abs(delta) <= maxDelta) return target;

    return current + (delta > 0 ? maxDelta : -maxDelta);
}

namespace Manhattan::Core {
    NavigatorController::NavigatorController(const App &app) : BaseController(app),
    _kinematics(app.GetController<RobotOdometry>()->GetKinematics()),
    _angularPid(angularKp, angularKi, angularKd) {

        _motor = app.GetController<MotorController>();
        _slam = app.GetController<SlamController>();

        _pathPublisher = _node->create_publisher<nav_msgs::msg::Path>("~/nav/desired_path", 10);
        _rayCastPublisher = _node->create_publisher<visualization_msgs::msg::MarkerArray>("~/nav/ray_cast", 10);

        _timer = _node->create_wall_timer(
            10ms, // todo: timer frequency config duplication
            [this] { Update(); }
        );
    }

    void NavigatorController::SetPath(std::queue<GridCell*> path) {
        _path = queue(path);
    }

    void NavigatorController::PublishPath() const {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = _node->now();

        auto poseMsg = geometry_msgs::msg::PoseStamped();
        poseMsg.header.stamp = msg.header.stamp;
        poseMsg.header.frame_id = "map";

        auto position = _slam->CurrentPose().position;
        poseMsg.pose.position.x = position.x;
        poseMsg.pose.position.y = position.y;
        poseMsg.pose.position.z = 0.0;

        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.pose.orientation.w = 1.0;

        msg.poses.push_back(poseMsg);

        auto temp = _path;
        while (!temp.empty()) {

            position = temp.front()->GetWorldPosition();
            poseMsg.pose.position.x = position.x;
            poseMsg.pose.position.y = position.y;
            poseMsg.pose.position.z = 0.0;

            msg.poses.push_back(poseMsg);

            temp.pop();
        }

        _pathPublisher->publish(msg);
    }

    std::queue<GridCell*> NavigatorController::CalculatePath(GridCell* destination) const {
        struct QueueItem {
            GridCell* cell;
            double distance;

            bool operator>(const QueueItem& other) const {
                return distance > other.distance;
            }
        };

        std::map<GridCell*, double> distances;
        std::map<GridCell*, GridCell*> previous;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<>> openSet;
        std::unordered_set<GridCell*> visited;

        const auto startCell = _slam->GetCell(_slam->CurrentPose().position);
        if (startCell == nullptr || destination == nullptr) return {};

        if (startCell == destination) return {};

        distances[startCell] = 0.0;
        openSet.push({startCell, 0.0});

        while (!openSet.empty()) {
            const auto current = openSet.top().cell;
            openSet.pop();

            if (current == nullptr) continue;
            if (visited.contains(current)) continue;
            visited.insert(current);

            if (current == destination) break;

            for (GridCell* neighbor : _slam->GetNeighbors(current)) {
                if (neighbor == nullptr) continue;
                if (neighbor->IsOccupied()) continue;

                const auto alt = distances[current] + neighbor->GetCost();

                const auto distIterator = distances.find(neighbor);
                if (distIterator != distances.end() && alt >= distIterator->second) continue;

                distances[neighbor] = alt;
                previous[neighbor] = current;
                openSet.push({neighbor, alt});
            }
        }

        if (!distances.contains(destination)) return {};

        std::vector<GridCell*> path;
        auto current = destination;

        while (current != nullptr && current != startCell) {
            path.push_back(current);

            const auto it = previous.find(current);
            if (it == previous.end()) {
                return {};
            }

            current = it->second;
        }

        std::reverse(path.begin(), path.end());

        std::queue<GridCell*> result;
        for (auto* cell : path) {
            result.push(cell);
        }

        return result;
    }

    std::vector<GridCell*> NavigatorController::SmoothPath(const std::vector<GridCell*>& pathList) const {
        std::vector<GridCell*> smoothedPath;
        if (pathList.empty()) return smoothedPath;

        smoothedPath.push_back(pathList.front());
        size_t currentIndex = 0;

        for (size_t i = 1; i < pathList.size(); ++i) {
            if (HasLineOfSight(pathList[currentIndex], pathList[i])) continue;

            smoothedPath.push_back(pathList[i - 1]);
            currentIndex = i - 1;
        }

        if (smoothedPath.back() != pathList.back()) {
            smoothedPath.push_back(pathList.back());
        }

        return smoothedPath;
    }

    bool NavigatorController::HasLineOfSight(GridCell* start, GridCell* end) const {
        const auto startPos = start->GetWorldPosition();
        const auto endPos = end->GetWorldPosition();
        const auto distance = Vector2::Distance(startPos, endPos);
        const auto direction = (endPos - startPos).Normalized();

        // 0.03 is grid cell size
        const auto step = 0.03 / 2.0;
        const auto perpendicular = Vector2(-direction.y, direction.x) * 0.1;

        for (double d = step; d < distance; d += step) {
            const auto centerPos = startPos + direction * d;

            if (IsBlocking(centerPos) ||
                IsBlocking(centerPos + perpendicular) ||
                IsBlocking(centerPos - perpendicular)) {
                return false;
                }
        }
        return true;
    }

    bool NavigatorController::IsBlocking(const Vector2& position) const {
        const auto cell = _slam->GetCell(position);
        return cell == nullptr || cell->IsOccupied();
    }


    void NavigatorController::ClearPath() {
        _path = {};
    }

    bool NavigatorController::HasPath() const {
        return !_path.empty();
    }

    vector<RayHit> NavigatorController::RayCastAround(const Pose &pose) const {
        vector<RayHit> hits;

        auto angle = pose.rotation + M_PI * 0.5f;
        const auto angleStep = M_PI * 2 / rayCount;

        for (auto i = 0; i < rayCount; i++) {
            RayHit rayHit;
            const auto hit = _slam->RayCast(pose.position,
                Vector2(cos(angle), sin(angle)),
                rayHit, rayDistance
            );

            angle += angleStep;
            if (!hit) continue;

            hits.push_back(rayHit);
        }

        return hits;
    }

    void NavigatorController::PublishRayCast(const vector<RayHit> &hits, const Pose &pose, const Vector2 &desiredDirection) const {
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

    Vector2 NavigatorController::GetDirection(const vector<RayHit> &rayHits, const Pose &pose, const Vector2 &desiredDirection) const {
        auto direction = desiredDirection;
        const auto rayWeight = 1.0 / rayCount;

        for (const auto rayHit : rayHits) {
            const auto dst = Vector2::Distance(pose.position, rayHit.hit);

            if (dst >= avoidanceDistance) continue;

            const auto proximity = 1.0 - dst / avoidanceDistance;
            const auto pushForce = proximity * proximity * avoidanceStrength;

            direction = direction + rayHit.normal * (pushForce * rayWeight);
        }

        return direction.Normalized();
    }

    void NavigatorController::Update() {
        if (!HasPath())
        {
            const auto speed = _kinematics.inverse(RobotSpeed { 0, 0 });

            _motor->SetForce(speed.left, speed.right);
            return;
        }

        PublishPath();

        auto pose = _slam->CurrentPose();
        auto currentWaypoint = _path.front();
        auto directionToWaypoint = (currentWaypoint->GetWorldPosition() - pose.position).Normalized();

        if (Vector2::Distance(pose.position, currentWaypoint->GetWorldPosition()) < waypointTolerance)
        {
            _path.pop();
            return;
        }

        const auto rayHits = RayCastAround(pose);
        const auto desiredDirection = GetDirection(rayHits, pose, directionToWaypoint);

        PublishRayCast(rayHits, pose, desiredDirection);


        const auto angleToTarget = Vector2::SignedAngle(pose.forward, desiredDirection);
        const auto angularSpeedTarget = clamp(_angularPid.step(angleToTarget, updateDeltaTime), -maxAngularSpeed, maxAngularSpeed);
        _currentAngularVelocity = angularSpeedTarget;

        auto distanceFactor = 1.0;
        if (!rayHits.empty())
        {
            const auto forwardHit = rayHits[0];
            const auto distanceAhead = Vector2::Distance(forwardHit.hit, pose.position);

            distanceFactor = clamp(distanceAhead / rayDistance, 0.0, 1.0);
        }

        const auto turnFactor = clamp(exp(-turnDeceleration * abs(angleToTarget)), 0.0, 1.0);
        const auto targetSpeed = maxLinearSpeed * turnFactor * distanceFactor;

        _currentLinearVelocity = MoveTowards(_currentLinearVelocity, targetSpeed,
            (targetSpeed > _currentLinearVelocity ? acceleration : deceleration) * 0.1 * maxLinearSpeed
        );

        const auto speed = _kinematics.inverse(RobotSpeed { _currentLinearVelocity, _currentAngularVelocity });
        _motor->SetForce(speed.left, speed.right);
    }


}


