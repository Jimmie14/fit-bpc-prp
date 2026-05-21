#pragma once

#include "Common/RosEngine.hpp"
#include "Math/Pose.hpp"
#include "Nav/GridMap.hpp"
#include "Nav/MazeGraph.hpp"

namespace Manhattan::core {
class MazePathPlannerEngine : public RosEngine {
public:
    explicit MazePathPlannerEngine(const App& app);

protected:
    void OnEnable() override;
    void OnDisable() override;

private:

    TimerBase::SharedPtr _timer;

    mutex _lock;

    math::Pose _pose;
    nav::GridMap _map{};
    nav::MazeGraph _graph;
    std::optional<nav::GraphPosition> _target;

    nav::NodeId _lastNodeId;
    nav::NodeId _currentNodeId;
    nav::NodeId _nextNodeId;

    bool _initialized = false;

    void planPath();
};
}

