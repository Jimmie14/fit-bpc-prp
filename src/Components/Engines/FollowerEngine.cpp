#include "FollowerEngine.hpp"

using namespace std;

namespace Manhattan::Core {
FollowerEngine::FollowerEngine(const App& app)
    : RosEngine(app, "follower")
    , _fov(120)
    , _rayDistance(8)
    , _rayCount(21)
    , _avoidanceDistance(0.1)
{
    _map = app.GetComponent<MappingEngine>();
    _navigator = app.GetComponent<NavigatorEngine>();
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

void FollowerEngine::FollowCorridor()
{
    if (!_navigator->IsInDestination())
        return;

    const auto pose = _map->CurrentPose();
    const auto target = GetTarget(pose);
    const auto targetCell = _map->GetCell(target);

    if (targetCell == nullptr)
        return;

    _navigator->SetDestination(targetCell);
}

Vector2 FollowerEngine::GetTarget(const Pose& pose) const
{
    const auto rad = _fov * (M_PI / 180.0);
    auto angle = (M_PI - rad) * 0.5;
    auto step = rad / (_rayCount - 1);

    auto dst = 0.0;
    auto pos = pose.position;

    for (auto i = 0; i < _rayCount; i++) {
        auto ray = RayHit();
        auto direction = Vector2(cos(pose.rotation + angle), sin(pose.rotation + angle));
        auto hit = _map->RayCast(pose.position, direction, ray, _rayDistance);

        angle += step;

        auto hitPoint = hit ? ray.hit : pose.position + direction * _rayDistance;
        auto dstToHit = Vector2::Distance(pose.position, hitPoint);

        if (!hit) {
            std::cout << "Ray not hit " << i << std::endl;
        }

        if (dstToHit <= dst)
            continue;

        dst = dstToHit;
        pos = hitPoint + ray.normal * _avoidanceDistance;
    }

    return pos;
}

void FollowerEngine::Update()
{
    FollowCorridor();
}
} // namespace Manhattan::Core