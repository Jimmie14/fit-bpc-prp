#include "NavigatorGraphBuilder.hpp"

namespace Manhattan::Core {

NavigatorGraphBuilder::NavigatorGraphBuilder(const App &app)
    : RosConnector(app)
{
    _mappingEngine = app.GetController<MappingEngine>();
    _markerPublisher = _node->create_publisher<visualization_msgs::msg::MarkerArray>("~/nav_graph", 10);
}

void NavigatorGraphBuilder::BuildGraph(float costThreshold)
{
    int w = _mappingEngine->GetWidth();
    int h = _mappingEngine->GetHeight();

    std::vector binary(w * h, false);
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            auto* cell = _mappingEngine->GetCell(Vector2Int(x, y));
            binary[y * w + x] = (cell != nullptr && !cell->IsOccupied() && !cell->IsUnknown() && cell->GetCost() < costThreshold);
        }
    }

    // 2. Skeletonize
    auto skeleton = ZhangSuenThinning(binary, w, h);

    // 3. Build Graph
    _graphNodes.clear();
    std::map<std::pair<int, int>, std::shared_ptr<Node>> nodeDict;

    std::vector<Vector2Int> dirs = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    };

    for (int x = 1; x < w - 1; x++) {
        for (int y = 1; y < h - 1; y++) {
            if (!skeleton[y * w + x]) continue;

            int neighbors = 0;
            for (const auto& d : dirs) {
                if (skeleton[(y + d.y) * w + (x + d.x)]) neighbors++;
            }

            if (neighbors != 2) {
                auto node = std::make_shared<Node>();
                node->gridPosition = Vector2Int(x, y);
                node->worldPosition = _mappingEngine->GridToWorld(node->gridPosition);
                _graphNodes.push_back(node);
                nodeDict[{x, y}] = node;
            }
        }
    }

    // Build edges
    for (auto& node : _graphNodes) {
        for (const auto& d : dirs) {
            Vector2Int current = node->gridPosition + d;
            if (!InBounds(current.x, current.y, w, h) || !skeleton[current.y * w + current.x]) continue;

            Vector2Int prev = node->gridPosition;
            std::vector<Vector2> path;

            while (true) {
                path.push_back(_mappingEngine->GridToWorld(current));

                if (nodeDict.count({current.x, current.y})) {
                    auto target = nodeDict[{current.x, current.y}];
                    if (target != node) {
                        auto edge = std::make_shared<Edge>();
                        edge->from = node;
                        edge->to = target;
                        edge->path = path;
                        node->connections.push_back(edge);
                    }
                    break;
                }

                Vector2Int next(0, 0);
                bool found = false;
                for (const auto& nd : dirs) {
                    Vector2Int cand = current + nd;
                    if (cand.x == prev.x && cand.y == prev.y) continue;
                    if (InBounds(cand.x, cand.y, w, h) && skeleton[cand.y * w + cand.x]) {
                        next = cand;
                        found = true;
                        break;
                    }
                }

                if (!found) break;
                prev = current;
                current = next;
            }
        }
    }
}

std::vector<bool> NavigatorGraphBuilder::ZhangSuenThinning(const std::vector<bool>& img, int w, int h)
{
    std::vector<bool> skeleton = img;
    bool changed;
    do {
        changed = false;
        std::vector<std::pair<int, int>> toRemove;

        for (int step = 0; step < 2; step++) {
            toRemove.clear();
            for (int x = 1; x < w - 1; x++) {
                for (int y = 1; y < h - 1; y++) {
                    if (!skeleton[y * w + x]) continue;

                    int B = CountNeighbors(skeleton, x, y, w, h);
                    int A = CountTransitions(skeleton, x, y, w, h);

                    bool p2 = skeleton[(y + 1) * w + x];
                    bool p4 = skeleton[y * w + (x + 1)];
                    bool p6 = skeleton[(y - 1) * w + x];
                    bool p8 = skeleton[y * w + (x - 1)];

                    bool condition;
                    if (step == 0)
                        condition = !(p2 && p4 && p6) && !(p4 && p6 && p8);
                    else
                        condition = !(p2 && p4 && p8) && !(p2 && p6 && p8);

                    if (B >= 2 && B <= 6 && A == 1 && condition) {
                        toRemove.push_back({x, y});
                        changed = true;
                    }
                }
            }
            for (auto& p : toRemove) skeleton[p.second * w + p.first] = false;
        }
    } while (changed);
    return skeleton;
}

int NavigatorGraphBuilder::CountNeighbors(const std::vector<bool>& img, int x, int y, int w, int h)
{
    int count = 0;
    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++)
            if ((i != 0 || j != 0) && img[(y + j) * w + (x + i)]) count++;
    return count;
}

int NavigatorGraphBuilder::CountTransitions(const std::vector<bool>& img, int x, int y, int w, int h)
{
    bool p[8] = {
        img[(y + 1) * w + x], img[(y + 1) * w + (x + 1)], img[y * w + (x + 1)], img[(y - 1) * w + (x + 1)],
        img[(y - 1) * w + x], img[(y - 1) * w + (x - 1)], img[y * w + (x - 1)], img[(y + 1) * w + (x - 1)]
    };
    int transitions = 0;
    for (int i = 0; i < 8; i++)
        if (!p[i] && p[(i + 1) % 8]) transitions++;
    return transitions;
}

bool NavigatorGraphBuilder::InBounds(int x, int y, int w, int h) const {
    return x >= 0 && y >= 0 && x < w && y < h;
}

void NavigatorGraphBuilder::PublishMarkers() {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker nodesMarker;
    nodesMarker.header.frame_id = "map";
    nodesMarker.header.stamp = _node->now();
    nodesMarker.ns = "graph_nodes";
    nodesMarker.id = 0;
    nodesMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodesMarker.scale.x = nodesMarker.scale.y = nodesMarker.scale.z = _nodeSize;
    nodesMarker.color.r = 1.0; nodesMarker.color.a = 1.0;

    visualization_msgs::msg::Marker edgesMarker;
    edgesMarker.header.frame_id = "map";
    edgesMarker.header.stamp = _node->now();
    edgesMarker.ns = "graph_edges";
    edgesMarker.id = 1;
    edgesMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edgesMarker.scale.x = 0.02;
    edgesMarker.color.g = 1.0; edgesMarker.color.a = 0.8;

    for (const auto& node : _graphNodes) {
        geometry_msgs::msg::Point p;
        p.x = node->worldPosition.x; p.y = node->worldPosition.y; p.z = 0.05;
        nodesMarker.points.push_back(p);

        for (const auto& edge : node->connections) {
            for (size_t i = 0; i < edge->path.size(); ++i) {
                geometry_msgs::msg::Point p1;
                p1.x = (i == 0) ? node->worldPosition.x : edge->path[i-1].x;
                p1.y = (i == 0) ? node->worldPosition.y : edge->path[i-1].y;
                p1.z = 0.02;

                geometry_msgs::msg::Point p2;
                p2.x = edge->path[i].x; p2.y = edge->path[i].y; p2.z = 0.02;

                edgesMarker.points.push_back(p1);
                edgesMarker.points.push_back(p2);
            }
        }
    }

    markers.markers.push_back(nodesMarker);
    markers.markers.push_back(edgesMarker);
    _markerPublisher->publish(markers);
}

} // namespace Manhattan::Core