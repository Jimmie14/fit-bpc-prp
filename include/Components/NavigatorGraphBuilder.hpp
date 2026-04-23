#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "Vector2.hpp"
#include "MappingEngine.hpp"

namespace Manhattan::Core {

struct Node;

struct Edge {
    std::shared_ptr<Node> from;
    std::shared_ptr<Node> to;
    std::vector<Vector2> path;
};

struct Node {
    Vector2Int gridPosition;
    Vector2 worldPosition;
    std::vector<std::shared_ptr<Edge>> connections;
};

class NavigatorGraphBuilder : RosConnector {
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

    std::vector<std::shared_ptr<Node>> _graphNodes;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _markerPublisher;

    float _nodeSize = 0.1f;
};

} // namespace Manhattan::Core