#pragma once

#include "Common/RosEngine.hpp"
#include "Nav/Grid.hpp"
#include "Nav/GridMap.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace Manhattan::core {
class MazeGraphEngine : public RosEngine {
public:
    explicit MazeGraphEngine(const App& app);

protected:

    TimerBase::SharedPtr _timer;


    void OnEnable() override;
    void OnDisable() override;

private:
    struct Node {
        math::Vector2Int cell;
    };


    unordered_map<math::Vector2Int, Node, math::Vector2IntHash> _nodes;
    vector<vector<math::Vector2Int>> _paths;

    nav::Grid<bool> _skeleton;
    nav::GridMap _map;


    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _debugPublisher;

    vector<math::Vector2Int> pathToClosestNode(const math::Vector2Int& cell, int radius);

    void createDeadEnds();


    void update();
    void publish() const;
};
}
