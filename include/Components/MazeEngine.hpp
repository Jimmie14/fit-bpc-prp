#pragma once

#include "RosEngine.hpp"
#include "NavigatorEngine.hpp"
#include "NavigatorGraphBuilder.hpp"

namespace Manhattan::Core {

class MazeEngine : public RosEngine {
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
        bool visited;
        Vector2 position;
        std::vector<std::shared_ptr<WayPoint>> connected;

        [[nodiscard]] std::shared_ptr<WayPoint> GetInDirection(const Direction dir, const Vector2& forward) const
        {
            for (auto point : connected) {
                const auto dirToWaypoint = (point->position - position).Normalized();
                const auto dot = Vector2::Dot(dirToWaypoint, forward);
                const auto perpDot = dirToWaypoint.x * forward.y - dirToWaypoint.y * forward.x;


                switch (dir) {
                    case Direction::Left:
                        if (perpDot > 0.5)
                            return point;
                        break;
                    case Direction::Right:
                        if (perpDot < -0.5)
                            return point;
                        break;
                    case Direction::Forward:
                        if (dot > 0.5)
                            return point;
                        break;
                    case Direction::Back:
                        if (dot < -0.5)
                            return point;
                        break;
                }
            }

            return nullptr;
        }
    };

    void Update();

    std::shared_ptr<WayPoint> NextJunction(std::shared_ptr<WayPoint> current);

    std::shared_ptr<NavigatorEngine> _navigator;
    std::shared_ptr<MappingEngine> _mapping;

    std::shared_ptr<WayPoint> _currentWayPoint = nullptr;

    TimerBase::SharedPtr _timer;
    TimerBase::SharedPtr _initialTimer;
};

} // namespace Manhattan::Core