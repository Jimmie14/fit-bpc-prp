#pragma once

#include "Nav/Grid.hpp"
#include "NavigatorEngine.hpp"
#include "NavigatorGraphBuilder.hpp"
#include "Common/RosEngine.hpp"

namespace Manhattan::core {

class MazeEngine final : public RosEngine {
public:
    explicit MazeEngine(const App& app);

    void OnEnable() override;
    void OnDisable() override;

private:
    enum class Direction {
        Left,
        Right,
        Forward,
        Back
    };

    struct WayPoint {
        struct Connection {
            shared_ptr<WayPoint> target;
            std::vector<Vector2> path;
        };

        bool visited;
        Vector2 position;
        std::vector<Connection> connected;

        [[nodiscard]] std::shared_ptr<WayPoint> GetInDirection(const Direction dir, const Vector2& forward) const
        {
            // for (auto point : connected) {
            //     const auto dirToWaypoint = (point.target->position - position).Normalized();
            //     const auto dot = Vector2::Dot(dirToWaypoint, forward);
            //     const auto perpDot = dirToWaypoint.x * forward.y - dirToWaypoint.y * forward.x;
            //
            //
            //     // switch (dir) {
            //     //     case Direction::Left:
            //     //         if (perpDot > 0.5)
            //     //             return point;
            //     //         break;
            //     //     case Direction::Right:
            //     //         if (perpDot < -0.5)
            //     //             return point;
            //     //         break;
            //     //     case Direction::Forward:
            //     //         if (dot > 0.5)
            //     //             return point;
            //     //         break;
            //     //     case Direction::Back:
            //     //         if (dot < -0.5)
            //     //             return point;
            //     //         break;
            //     // }
            // }

            return nullptr;
        }
    };

    void Update();

    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _poseSubscription;
    Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _mapSubscription;

    void OnPose(const geometry_msgs::msg::PoseStamped::SharedPtr& msg) const;
    void OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);

    void PublishCurrenThGraph() const;

    std::shared_ptr<WayPoint> NextJunction(const std::shared_ptr<WayPoint>& current);

    std::optional<Vector2Int> ClosestOnThinnedMap(const Vector2& position);

    std::vector<Vector2Int> GetValidNeighbors(const Vector2Int& cell);

    bool IsWaypoint(const Vector2Int& cell);
    std::shared_ptr<WayPoint> WalkUntilWaypoint(Vector2Int prev, Vector2Int current);

    std::shared_ptr<WayPoint> Init();

    nav::Grid<bool> _thinned_map;

    std::shared_ptr<NavigatorEngine> _navigator;
    std::shared_ptr<MappingEngine> _mapping;

    std::shared_ptr<WayPoint> _currentWayPoint = nullptr;

    TimerBase::SharedPtr _timer;
    TimerBase::SharedPtr _initialTimer;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _graphPublisher;
};

} // namespace Manhattan::Core