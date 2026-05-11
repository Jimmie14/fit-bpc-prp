#include "Components/NavigatorGraphBuilder.hpp"

#include <bitset>

namespace Manhattan::core {

vector<bool> reduce2x2OR(const vector<bool>& grid, int w, int h)
{
    int newW = w / 2;
    int newH = h / 2;

    vector<bool> out(newW * newH, false);

    for (int y = 0; y < newH; ++y) {
        for (int x = 0; x < newW; ++x) {

            bool val = grid[(2 * y) * w + (2 * x)] || grid[(2 * y) * w + (2 * x + 1)] || grid[(2 * y + 1) * w + (2 * x)] || grid[(2 * y + 1) * w + (2 * x + 1)];

            out[y * newW + x] = val;
        }
    }

    return out;
}

static int CountGroupedNeighbors(const std::vector<bool>& img, int x, int y, int w, int h)
{
    auto at = [&](const int dx, const int dy) -> bool {
        return img[(y + dy) * w + (x + dx)];
    };

    const uint8_t mask = (at(0, -1) ? 1 << 0 : 0) | (at(1, -1) ? 1 << 1 : 0) | (at(1, 0) ? 1 << 2 : 0) | (at(1, 1) ? 1 << 3 : 0) | (at(0, 1) ? 1 << 4 : 0) | (at(-1, 1) ? 1 << 5 : 0) | (at(-1, 0) ? 1 << 6 : 0) | (at(-1, -1) ? 1 << 7 : 0);

    if (mask == 0) return 0;

    std::vector<int> runs;

    // Step 1: collect linear runs
    int i = 0;
    while (i < 8) {
        if (!(mask & (1 << i))) {
            i++;
            continue;
        }

        int start = i;
        while (i < 8 && (mask & (1 << i))) {
            i++;
        }
        runs.push_back(i - start);
    }

    if (runs.empty()) return 0;

    // Step 2: handle wrap-around merge
    const bool firstBit = mask & 1;
    const bool lastBit = mask & (1 << 7);

    if (firstBit && lastBit && runs.size() > 1) {
        runs.front() += runs.back();
        runs.pop_back();
    }

    // Step 3: compute result
    int result = 0;
    for (int len : runs) {
        result += (len <= 2) ? 1 : 2;
    }

    return result;
}

NavigatorGraphBuilder::NavigatorGraphBuilder(const App& app)
    : RosEngine(app, "navigator_graph_builder")
{
    _mappingEngine = app.getComponent<MappingEngine>();
    _markerPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("nav/graph", 1);
    _gridPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("nav/grid", 1);

    _publishTimer = create_wall_timer(100ms, [this] {
        BuildGraph();
        PublishMarkers();
    });
}

void NavigatorGraphBuilder::BuildGraph()
{
    int w = _mappingEngine->GetWidth();
    int h = _mappingEngine->GetHeight();

    std::vector<bool> binary(w * h, false);
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            auto* cell = _mappingEngine->GetCell(Vector2Int(x, y));
            binary[y * w + x] = (cell != nullptr && !cell->IsOccupied() && !cell->IsUnknown());
        }
    }

    // 2. Skeletonize
    auto skeleton = ZhangSuenThinning(binary, w, h);

    PublishGrid(skeleton, w, h);

    // 3. Build Graph
    _graphNodes.clear();
    std::map<std::pair<int, int>, std::shared_ptr<NavigatorNode>> nodeDict;

    std::vector<Vector2Int> dirs = {
        { 0, 1 },
        { 1, 0 },
        { 0, -1 },
        { -1, 0 },
        { 1, 1 },
        { 1, -1 },
        { -1, -1 },
        { -1, 1 }
    };

    for (int x = 1; x < w - 1; x++) {
        for (int y = 1; y < h - 1; y++) {
            if (!skeleton[y * w + x])
                continue;

            int neighbors = CountGroupedNeighbors(skeleton, x, y, w, h);

            if (neighbors != 2) {
                auto node = std::make_shared<NavigatorNode>();
                node->gridPosition = Vector2Int(x, y);
                node->worldPosition = _mappingEngine->GridToWorld(node->gridPosition);
                _graphNodes.push_back(node);
                nodeDict[{ x, y }] = node;
            }
        }
    }

    // Build edges
    for (auto& node : _graphNodes) {
        for (const auto& d : dirs) {
            Vector2Int current = node->gridPosition + d;
            if (!InBounds(current.x, current.y, w, h) || !skeleton[current.y * w + current.x])
                continue;

            Vector2Int prev = node->gridPosition;
            std::vector<Vector2> path;

            auto visited = vector<bool>(w * h);

            while (true) {
                const auto currentIndex = current.y * w + current.x;
                if (visited[currentIndex])
                    break;

                visited[currentIndex] = true;
                path.push_back(_mappingEngine->GridToWorld(current));

                if (nodeDict.count({ current.x, current.y })) {
                    auto target = nodeDict[{ current.x, current.y }];
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
                    if (cand.x == prev.x && cand.y == prev.y)
                        continue;
                    if (InBounds(cand.x, cand.y, w, h) && skeleton[cand.y * w + cand.x]) {
                        next = cand;
                        found = true;
                        break;
                    }
                }

                if (!found)
                    break;
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
                    if (!skeleton[y * w + x])
                        continue;

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
                        toRemove.push_back({ x, y });
                        changed = true;
                    }
                }
            }
            for (auto& p : toRemove)
                skeleton[p.second * w + p.first] = false;
        }
    } while (changed);
    return skeleton;
}

int NavigatorGraphBuilder::CountNeighbors(const std::vector<bool>& img, int x, int y, int w, int h)
{
    int count = 0;
    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++)
            if ((i != 0 || j != 0) && img[(y + j) * w + (x + i)])
                count++;
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
        if (!p[i] && p[(i + 1) % 8])
            transitions++;
    return transitions;
}

bool NavigatorGraphBuilder::InBounds(int x, int y, int w, int h) const
{
    return x >= 0 && y >= 0 && x < w && y < h;
}

void NavigatorGraphBuilder::PublishMarkers()
{
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker nodesMarker;
    nodesMarker.header.frame_id = "map";
    nodesMarker.header.stamp = now();
    nodesMarker.ns = "graph_nodes";
    nodesMarker.id = 0;
    nodesMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodesMarker.scale.x = nodesMarker.scale.y = nodesMarker.scale.z = _nodeSize;
    nodesMarker.color.r = 1.0;
    nodesMarker.color.a = 1.0;

    visualization_msgs::msg::Marker edgesMarker;
    edgesMarker.header.frame_id = "map";
    edgesMarker.header.stamp = now();
    edgesMarker.ns = "graph_edges";
    edgesMarker.id = 1;
    edgesMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edgesMarker.scale.x = 0.02;
    edgesMarker.color.g = 1.0;
    edgesMarker.color.a = 0.8;

    visualization_msgs::msg::Marker clearMarker;
    clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clearMarker);

    for (const auto& node : _graphNodes) {
        geometry_msgs::msg::Point p;
        p.x = node->worldPosition.x;
        p.y = node->worldPosition.y;
        p.z = 0.02;
        nodesMarker.points.push_back(p);

        for (const auto& edge : node->connections) {
            for (size_t i = 0; i + 1 < edge->path.size(); ++i) {
                geometry_msgs::msg::Point p1;
                p1.x = edge->path[i].x;
                p1.y = edge->path[i].y;
                p1.z = 0.02;

                geometry_msgs::msg::Point p2;
                p2.x = edge->path[i + 1].x;
                p2.y = edge->path[i + 1].y;
                p2.z = 0.02;

                edgesMarker.points.push_back(p1);
                edgesMarker.points.push_back(p2);
            }
        }
    }

    markers.markers.push_back(nodesMarker);
    markers.markers.push_back(edgesMarker);
    _markerPublisher->publish(markers);
}

void NavigatorGraphBuilder::PublishGrid(const vector<bool>& img, const int w, const int h)
{
    const auto cellSize = _mappingEngine->GetCellSize();

    nav_msgs::msg::OccupancyGrid gridMsg;
    gridMsg.header.stamp = now();
    gridMsg.header.frame_id = "map";

    gridMsg.info.origin.position.x = w * cellSize * -0.5;
    gridMsg.info.origin.position.y = h * cellSize * -0.5;
    gridMsg.info.origin.position.z = 0.0;

    gridMsg.info.origin.orientation.x = 0.0;
    gridMsg.info.origin.orientation.y = 0.0;
    gridMsg.info.origin.orientation.z = 0.0;
    gridMsg.info.origin.orientation.w = 1.0;

    gridMsg.info.width = w;
    gridMsg.info.height = h;
    gridMsg.info.resolution = static_cast<float>(cellSize);

    const auto size = gridMsg.info.width * gridMsg.info.height;
    gridMsg.data.resize(size);

    for (auto x = 0; x < gridMsg.info.width; x++) {
        for (auto y = 0; y < gridMsg.info.height; y++) {
            gridMsg.data[y * w + x] = img[y * w + x] ? 127 : -1;
        }
    }

    _gridPublisher->publish(gridMsg);
}

} // namespace Manhattan::Core