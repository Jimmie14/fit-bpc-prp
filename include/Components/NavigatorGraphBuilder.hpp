#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "MappingEngine.hpp"
#include "Math/Vector2.hpp"

namespace Manhattan::Core {

struct NavigatorNode;

struct Edge {
    std::shared_ptr<NavigatorNode> from;
    std::shared_ptr<NavigatorNode> to;
    std::vector<Vector2> path;
};

struct NavigatorNode {
    Vector2Int gridPosition;
    Vector2 worldPosition;
    std::vector<std::shared_ptr<Edge>> connections;
};

class NavigatorGraphBuilder final : public RosEngine {
public:
    NavigatorGraphBuilder(const App& app);

    void BuildGraph();

    std::vector<std::shared_ptr<NavigatorNode>> GetNodes() const { return _graphNodes; }

private:
    std::vector<bool> ZhangSuenThinning(const std::vector<bool>& binary, int w, int h);
    int CountNeighbors(const std::vector<bool>& img, int x, int y, int w, int h);
    int CountTransitions(const std::vector<bool>& img, int x, int y, int w, int h);
    bool InBounds(int x, int y, int w, int h) const;

    TimerBase::SharedPtr _publishTimer;

    std::shared_ptr<MappingEngine> _mappingEngine;

    std::vector<std::shared_ptr<NavigatorNode>> _graphNodes;
    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _markerPublisher;
    Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _gridPublisher;

    float _nodeSize = 0.05f;

    void PublishMarkers();
    void PublishGrid(const vector<bool>& img, const int w, const int h);
};

} // namespace Manhattan::Core