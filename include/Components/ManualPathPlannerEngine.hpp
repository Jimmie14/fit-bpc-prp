#pragma once

#include "Common/RosEngine.hpp"
#include "Math/Pose.hpp"
#include "Nav/GridMap.hpp"
#include "Nav/MazeGraph.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace Manhattan::core {
class ManualPathPlannerEngine : public RosEngine {
public:
    explicit ManualPathPlannerEngine(const App& app);

protected:
    void OnEnable() override;
    void OnDisable() override;

private:
    mutex _lock;

    Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _pointSubscription;

    math::Pose _pose;
    nav::GridMap _map {};
    nav::MazeGraph _graph;
    std::optional<nav::GraphPosition> _target;

    void planPath();
};
}
