#include "FollowerController.hpp"

using namespace std;

namespace Manhattan::Core
{
    FollowerController::FollowerController(const App& app) : BaseController(app),
        _fov(180), _rayDistance(15), _rayCount(7), _avoidanceDistance(0.2f)
    {
        _map = app.GetController<SlamController>();
        _navigator = app.GetController<NavigatorController>();

        Enable();
    }

    void FollowerController::OnEnable()
    {
        _startPosition = _map->CurrentPose().Position;

        _timer = _node->create_wall_timer(
            100ms,
            [this] { Update(); }
        );
    }

    void FollowerController::OnDisable()
    {
        _timer.reset();
    }

    void FollowerController::FollowCorridor() {
        if (_navigator->HasPath()) return;

        const auto pose = _map->CurrentPose();
        const auto target = GetTarget(pose);
        const auto targetCell = _map->GetCell(target);

        if (targetCell == nullptr) return;

        auto path = _navigator->CalculatePath(targetCell);
        _navigator->SetPath(path);
    }

    Vector2 FollowerController::GetTarget(const Pose& pose) const
    {
        const auto rad = _fov * (M_PI / 180.0);
        auto angle = (M_PI - rad) * 0.5;
        auto step = rad / (_rayCount - 1);

        auto dst = 0.0;
        auto pos = pose.Position;

        for (auto i = 0; i < _rayCount; i++) {
            auto ray = RayHit();
            auto direction = Vector2(cos(pose.Rotation + angle), sin(pose.Rotation + angle));
            auto hit = _map->RayCast(pose.Position, direction, ray, _rayDistance);

            angle += step;

            auto hitPoint = hit ? ray.hit : pose.Position + direction * _rayDistance;
            auto dstToHit = Vector2::Distance(pose.Position, hitPoint);

            if (dstToHit <= dst) continue;
            dst = dstToHit;
            pos = hitPoint + ray.normal * _avoidanceDistance;
        }

        return pos;
    }

    void FollowerController::Update()
    {
        FollowCorridor();
    }
}