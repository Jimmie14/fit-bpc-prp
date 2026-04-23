#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "Vector2.hpp"
#include "MappingEngine.hpp"

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
    NavigatorGraphBuilder(const App &app);

    void BuildGraph(float costThreshold = 10.0f);
    void PublishMarkers();

private:
    std::vector<bool> ZhangSuenThinning(const std::vector<bool>& binary, int w, int h);
    int CountNeighbors(const std::vector<bool>& img, int x, int y, int w, int h);
    int CountTransitions(const std::vector<bool>& img, int x, int y, int w, int h);
    bool InBounds(int x, int y, int w, int h) const;

    std::shared_ptr<MappingEngine> _mappingEngine;

    std::vector<std::shared_ptr<NavigatorNode>> _graphNodes;
    Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _markerPublisher;

    float _nodeSize = 0.1f;
};

} // namespace Manhattan::Core