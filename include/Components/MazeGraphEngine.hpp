#pragma once

#include "Common/RosEngine.hpp"
#include "Nav/Grid.hpp"
#include "Nav/GridMap.hpp"
#include "Nav/MazeGraph.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace Manhattan::core {

using namespace Manhattan::math;

class MazeGraphEngine : public RosEngine {
public:
    explicit MazeGraphEngine(const App& app);

protected:
    TimerBase::SharedPtr _timer;

    void OnEnable() override;
    void OnDisable() override;

private:
    nav::NodeId _currentNodeId = 0;
    nav::MazeGraph _graph;

    nav::Grid<bool> _skeleton;
    nav::GridMap _map;

    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _debugPublisher;

    vector<vector<Vector2i>> followPath(set<Vector2i>& visited, vector<Vector2i>& path);

    vector<Vector2i> pathToClosestNode(const Vector2i& cell, int radius);

    Vector2i findStartCandidate() const;

    nav::NodeId getNodeId(const nav::MazeGraph& oldGraph, const Vector2i& cell);

    void update();
    void publish() const;
};
}
