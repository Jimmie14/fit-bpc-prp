#include "FollowerEngine.hpp"

using namespace std;

namespace Manhattan::Core {
FollowerEngine::FollowerEngine(const App& app)
    : RosEngine(app, "follower")
    , _fov(180)
    , _rayDistance(4)
    , _rayCount(11)
    , _avoidanceDistance(0.2)
{
    _map = app.GetComponent<MappingEngine>();
    _navigator = app.GetComponent<NavigatorEngine>();

    _rayCastPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("nav/ray_cast", 1);
}

void FollowerEngine::OnEnable()
{
    _startPosition = _map->CurrentPose().position;

    _initialTimer = create_wall_timer(1s, [this]() {
        _initialTimer->cancel();

        _timer = create_wall_timer(100ms, [this] { Update(); });
    });
}

void FollowerEngine::OnDisable()
{
    _timer.reset();
    _initialTimer.reset();
}

void FollowerEngine::PublishRayCast(const vector<RayHit>& hits, const Pose& pose) const
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

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
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

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    markerArray.markers.push_back(marker);

    _rayCastPublisher->publish(markerArray);
}

void FollowerEngine::FollowCorridor()
{
    if (!_navigator->IsInDestination())
        return;

    const auto pose = _map->CurrentPose();

    double alignmentAdjust = 0.0;
    RayHit leftRay, rightRay;
    const auto leftDir = Vector2::FromAngle(pose.rotation + M_PI_2);
    const auto rightDir = Vector2::FromAngle(pose.rotation - M_PI_2);

    const bool hitL = _map->RayCast(pose.position, leftDir, leftRay, 2.0);
    const bool hitR = _map->RayCast(pose.position, rightDir, rightRay, 2.0);

    if (hitL && hitR) {
        auto error = Vector2::Distance(leftRay.hit, pose.position) - Vector2::Distance(rightRay.hit, pose.position);
        alignmentAdjust = error * 0.5;
    }

    // Pass the adjusted rotation to GetTarget
    const auto target = GetTarget(pose, pose.rotation); // + alignmentAdjust); // GetTarget(pose);
    const auto targetCell = _map->GetCell(target);

    if (targetCell == nullptr)
        return;

    _navigator->SetDestination(targetCell);
}

Vector2 FollowerEngine::GetTarget(const Pose& pose, double adjustedRotation) const
{
    const auto rad = _fov * (M_PI / 180.0);
    auto angle = (M_PI - rad) * 0.5;
    auto step = rad / (_rayCount - 1);

    auto dst = 0.0;
    auto pos = pose.position;

    std::vector<RayHit> rayHits;
    for (auto i = 0; i < _rayCount; i++) {
        auto ray = RayHit();
        auto direction = Vector2(cos(adjustedRotation + angle), sin(adjustedRotation + angle));
        auto hit = _map->RayCast(pose.position, direction, ray, _rayDistance);

        angle += step;

        auto hitPoint = hit ? ray.hit : pose.position + direction * _rayDistance;
        auto dstToHit = Vector2::Distance(pose.position, hitPoint);

        rayHits.push_back(ray);

        if (dstToHit <= dst)
            continue;
        dst = dstToHit;
        pos = hitPoint + ray.normal * _avoidanceDistance;
    }

    PublishRayCast(rayHits, pose);
    return pos;
}

void FollowerEngine::Update()
{
    FollowCorridor();
}
} // namespace Manhattan::Core