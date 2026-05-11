#pragma once

#include "Common/RosEngine.hpp"
#include "Nav/Grid.hpp"
#include "Nav/GridMap.hpp"

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
    struct Node {
        Vector2Int cell;
        bool isCrossing = false;
    };


    unordered_map<Vector2Int, Node, Vector2IntHash> _nodes;
    vector<vector<Vector2Int>> _paths;

    nav::Grid<bool> _skeleton;
    nav::GridMap _map;


    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _debugPublisher;

    vector<vector<Vector2Int>> followPath(set<Vector2Int>& visited, vector<Vector2Int>& path);

    vector<Vector2Int> pathToClosestNode(const Vector2Int& cell, int radius);

    Vector2Int findStartCandidate() const;


    void update();
    void publish() const;
};
}
