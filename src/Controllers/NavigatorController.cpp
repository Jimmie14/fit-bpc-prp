#include "NavigatorController.hpp"

#include "RobotOdometry.hpp"

using namespace std;

constexpr int rayCount = 64;
constexpr double rayDistance = 5.78;
constexpr double avoidanceDistance = 0.2;
constexpr double avoidanceStrength = 5;

constexpr double waypointTolerance = 0.15f;

constexpr double maxLinearSpeed = 5;
constexpr double maxAngularSpeed = 120;

constexpr double turnDeceleration = 6.0;
constexpr double acceleration = 0.1;
constexpr double deceleration = 0.8;

static double MoveTowards(const double current, const double target, const double maxDelta) {
    const auto delta = target - current;
    if (std::abs(delta) <= maxDelta) return target;

    return current + (delta > 0 ? maxDelta : -maxDelta);
}

namespace Manhattan::Core {
    NavigatorController::NavigatorController(const App &app) : BaseController(app),
    _kinematics(app.GetController<RobotOdometry>()->GetKinematics()) {
        _motor = app.GetController<MotorController>();
        _slam = app.GetController<SlamController>();

        _pathPublisher = _node->create_publisher<nav_msgs::msg::Path>("~/nav/desired_path", 10);
        //_rayCastPublisher = _node->create_publisher<visualization_msgs::msg::MarkerArray>("~/nav/ray_cast", 10);

        _timer = _node->create_wall_timer(
            100ms,
            [this] { Update(); }
        );
    }

    void NavigatorController::SetPath(std::queue<GridCell*> path) {
        _path = path;

        nav_msgs::msg::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = _node->now();

        auto temp = path;
        while (!temp.empty()) {

            auto poseMsg = geometry_msgs::msg::PoseStamped();
            poseMsg.header.stamp = _node->now();
            poseMsg.header.frame_id = "map";

            auto position = temp.front()->GetWorldPosition();
            poseMsg.pose.position.x = position.x;
            poseMsg.pose.position.y = position.y;
            poseMsg.pose.position.z = 0.0;

            poseMsg.pose.orientation.x = 0.0;
            poseMsg.pose.orientation.y = 0.0;
            poseMsg.pose.orientation.z = 0.0;
            poseMsg.pose.orientation.w = 1.0;

            msg.poses.push_back(poseMsg);

            temp.pop();
        }

        _pathPublisher->publish(msg);
    }

    std::queue<GridCell*> NavigatorController::CalculatePath(GridCell* destination) const {
        std::unordered_map<GridCell*, GridCell*> previous;
        std::vector<GridCell*> queue;

        auto startCell = _slam->GetCell(_slam->CurrentPose().position);
        if (startCell == nullptr) return { };

        queue.push_back(startCell);

        while (!queue.empty())
        {
            auto current = queue.front();
            queue.erase(queue.begin());

            if (current == destination) break;

            for (GridCell* neighbor : _slam->GetNeighbors(current))
            {
                if (previous.contains(neighbor)) continue;
                previous.emplace(neighbor, current);

                if (ranges::find(queue, neighbor) != queue.end()) continue;
                if (neighbor->IsOccupied()) continue;

                queue.push_back(neighbor);
            }
        }

        std::vector<GridCell*> path;
        auto current = destination;

        while (current != startCell)
        {
            path.push_back(current);

            auto it = previous.find(current);
            if (it == previous.end())
                break;

            current = it->second;
        }

        auto d = deque(path.rbegin(), path.rend());

        return std::queue(d);
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

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            markerArray.markers.push_back(marker);
        }

        _rayCastPublisher->publish(markerArray);

        return hits;
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

        const auto angleToTarget = Vector2::SignedAngle(pose.forward, desiredDirection);
        const auto angularSpeed = clamp(angleToTarget * 2.0, -maxAngularSpeed, maxAngularSpeed);

        auto distanceFactor = 1.0;
        if (!rayHits.empty())
        {
            const auto forwardHit = rayHits[0];
            const auto distanceAhead = Vector2::Distance(forwardHit.hit, pose.position);

            distanceFactor = clamp(distanceAhead / rayDistance, 0.0, 1.0);
        }

        const auto turnFactor = exp(-turnDeceleration * (abs(angleToTarget) / 180.0));
        const auto targetSpeed = maxLinearSpeed * turnFactor * distanceFactor;

        _currentLinearVelocity = MoveTowards(_currentLinearVelocity, targetSpeed,
            (targetSpeed > _currentLinearVelocity ? acceleration : deceleration) * 0.1 * maxLinearSpeed
        );

        const auto speed = _kinematics.inverse(RobotSpeed { _currentLinearVelocity, angularSpeed });
        _motor->SetForce(speed.left, speed.right);
    }


}


