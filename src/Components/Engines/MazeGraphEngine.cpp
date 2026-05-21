#include "Components/MazeGraphEngine.hpp"

#include "App.hpp"
#include "Components/MapThinningUnit.hpp"
#include "Math/Vec3.hpp"
#include "Messages/Nav.hpp"
#include "Viz/Marker.hpp"

#include <queue>

namespace Manhattan::core {

MazeGraphEngine::MazeGraphEngine(const App& app)
    : RosEngine(app, "maze_graph")
    , _skeleton(Grid<bool>::uninitialized())
    , _map()
{
    _debugPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("maze/graph", 1);

    _app.events->Subscribe<ThinnedMapEvent>([this](const ThinnedMapEvent& event) {
        _skeleton = event.grid;
        _map = GridMap(_skeleton);
    });
}

void MazeGraphEngine::OnEnable()
{
    _timer = create_wall_timer(500ms, [this] {
        update();
        publish();
    });
}

void MazeGraphEngine::OnDisable()
{
    _timer.reset();
}

static vector<Vector2i> getNeighbours(const Grid<bool>& skeleton, const Vector2i& cell)
{
    vector<Vector2i> result;

    for (auto dir : Vector2i::EightDirections()) {
        const auto neighbour = cell + dir;

        if (!skeleton.inBounds(neighbour.x, neighbour.y)) continue;
        if (!skeleton[neighbour]) continue;

        result.push_back(neighbour);
    }

    return result;
}

static bool isCrossroad(const Grid<bool>& skeleton, const Vector2i& cell)
{
    vector frontiers = { cell };
    vector<Vector2i> nextFrontiers;

    set<Vector2i> visited;

    for (int iteration = 0; iteration < 4; iteration++) {
        if (frontiers.empty()) return false;

        for (auto frontier : frontiers) {
            for (auto neighbour : getNeighbours(skeleton, frontier)) {
                if (visited.contains(neighbour)) continue;

                nextFrontiers.push_back(neighbour);
                visited.insert(neighbour);
            }
        }

        frontiers.clear();
        frontiers.swap(nextFrontiers);
    }

    return frontiers.size() > 2;
}

static vector<Vector2i> extractCommonPath(vector<vector<Vector2i>>& ways)
{
    vector<Vector2i> result;

    if (ways.empty()) return result;

    for (const auto& p : ways) {
        if (p.empty()) return result;
    }

    auto maxCommonLength = ways[0].size();
    for (const auto& way : ways | std::views::drop(1)) {
        if (way.size() > maxCommonLength) continue;

        maxCommonLength = way.size();
    }

    maxCommonLength--;

    int commonLength = 0;

    for (auto i = 0; i < maxCommonLength; i++) {
        const Vector2i& candidate = ways[0][i];

        const auto allMatch = std::ranges::all_of(ways | std::views::drop(1), [&](const auto& way) { return way[i] == candidate; });
        if (!allMatch) break;

        commonLength++;
    }

    if (commonLength == 0) return result;

    result.assign(ways[0].begin(), ways[0].begin() + commonLength);

    for (auto& way : ways) {
        way.erase(way.begin(), way.begin() + commonLength);
    }

    return result;
}

static int getCommonPathLength(const vector<Vector2i>& path1, const vector<Vector2i>& path2)
{
    auto length = 0;
    while (length < path1.size() && length < path2.size() && path1[length] == path2[length]) {
        length++;
    }

    return length;
}

static void filterSimilarPaths(vector<vector<Vector2i>>& ways)
{
    vector removed(ways.size(), false);

    for (auto i = 0; i < ways.size(); i++) {
        if (removed[i]) continue;

        for (auto j = i + 1; j < ways.size(); j++) {
            if (removed[j]) continue;

            const auto commonLength = getCommonPathLength(ways[i], ways[j]);

            const auto divergenceFirst = ways[i].size() - commonLength;
            const auto divergenceSecond = ways[j].size() - commonLength;

            if (divergenceFirst < 2) {
                removed[i] = true;
                continue;
            }

            if (divergenceSecond < 2) {
                removed[j] = true;
                continue;
            }
        }
    }

    vector<vector<Vector2i>> newWays;
    newWays.reserve(ways.size());

    for (auto i = 0; i < ways.size(); i++) {
        if (removed[i]) continue;

        newWays.push_back(ways[i]);
    }

    ways.swap(newWays);
}

vector<vector<Vector2i>> MazeGraphEngine::followPath(set<Vector2i>& visited, vector<Vector2i>& path)
{
    vector<vector<Vector2i>> ways;
    vector<vector<Vector2i>> newWays;

    auto lookaheadVisited = visited;

    const auto start = path.back();
    auto end = start;

    path.pop_back();

    ways.push_back(vector { start });

    while (!ways.empty()) {
        for (auto& way : ways) {
            const auto frontier = way.back();

            for (auto neighbour : getNeighbours(_skeleton, frontier)) {
                if (lookaheadVisited.contains(neighbour)) continue;

                lookaheadVisited.insert(neighbour);

                auto newWay = way;
                newWay.push_back(neighbour);

                newWays.push_back(newWay);
            }
        }

        ways.swap(newWays);
        newWays.clear();

        if (const auto commonWay = extractCommonPath(ways); !commonWay.empty()) {
            path.insert(path.end(), commonWay.begin(), commonWay.end());
            visited.insert(commonWay.begin(), commonWay.end());
        }

        if (ways.size() == 1) end = ways[0].back();

        const auto endPath = ranges::find_if(ways, [&](const auto& w) {
            const auto cell = w.back();

            return cell != start && _graph.containsNode(cell);
        });

        if (endPath != ways.end()) {
            path.insert(path.end(), endPath->begin(), endPath->end());
            visited.insert(endPath->begin(), endPath->end());
            return {};
        }

        const auto foundCrossroad = ranges::any_of(ways, [](const auto& w) { return w.size() > 5; });
        if (foundCrossroad) {
            filterSimilarPaths(ways);

            for (auto& way : ways) {
                way.insert(way.begin(), path.back());
            }

            return ways;
        }
    }

    path.insert(path.end(), end);

    return {};
}

vector<Vector2i> MazeGraphEngine::pathToClosestNode(const Vector2i& start, const int radius)
{
    vector<vector<Vector2i>> ways;
    vector<vector<Vector2i>> newWays;
    set<Vector2i> visited;

    ways.push_back({ start });

    while (true) {
        for (auto& way : ways) {
            const auto frontier = way.back();

            for (auto neighbour : getNeighbours(_skeleton, frontier)) {
                if (visited.contains(neighbour)) continue;

                if (_graph.containsNode(frontier)) return way;

                visited.insert(neighbour);

                auto newWay = way;
                newWay.push_back(neighbour);

                newWays.push_back(newWay);
            }
        }

        ways.swap(newWays);
        newWays.clear();

        const auto isOnePathTooLong = std::ranges::any_of(ways, [&](const auto& w) { return w.size() >= radius; });
        if (isOnePathTooLong) break;
    }

    return {};
}

static void clearVisitedAround(const Grid<bool>& skeleton, set<Vector2i>& visited, const Vector2i& start, const int radius)
{
    set<Vector2i> closed;
    vector<Vector2i> frontiers = { start };
    vector<Vector2i> nextFrontiers;

    for (auto i = 0; i < radius; i++) {
        for (auto frontier : frontiers) {
            for (auto neighbour : getNeighbours(skeleton, frontier)) {
                if (closed.contains(neighbour)) continue;

                nextFrontiers.push_back(neighbour);
                visited.erase(neighbour);
                closed.insert(neighbour);
            }
        }

        frontiers.swap(nextFrontiers);
        nextFrontiers.clear();
    }
}

Vector2i MazeGraphEngine::findStartCandidate() const
{
    auto min = std::numeric_limits<double>::max();
    Vector2i result = _map.worldToCoord(vec3::zero);

    const auto start = Vector2(result);

    for (int i = 0; i < _skeleton.size(); i++) {
        if (!_skeleton[i]) continue;

        const auto cell = Vector2i(_skeleton.indexToCoord(i));

        const auto neighbourCount = getNeighbours(_skeleton, cell).size();
        if (neighbourCount != 1) continue;

        const auto distance = Vector2::distance(start, Vector2(cell));
        if (distance > min) continue;

        min = distance;
        result = cell;
    }

    return result;
}

nav::NodeId MazeGraphEngine::getNodeId(const MazeGraph& oldGraph, const Vector2i& cell)
{
    const auto id = oldGraph.findClosestNode(cell, 5);
    if (id) return id.value();

    return ++_currentNodeId;
}

void MazeGraphEngine::update()
{
    if (_skeleton.empty()) return;

    const auto oldGraph = _graph;

    _graph = MazeGraph();

    set<Vector2i> visited;

    const auto start = findStartCandidate();
    const auto startNodeId = getNodeId(oldGraph, start);

    _graph.createNode(startNodeId, start);

    queue<pair<NodeId, vector<Vector2i>>> queue;
    queue.push({ startNodeId, { start } });

    while (!queue.empty()) {
        auto [startNode, path] = queue.front();
        queue.pop();

        auto ways = followPath(visited, path);
        if (path.size() <= 2) continue;

        for (auto point : ways | views::join) {
            visited.insert(point);
        }

        auto endCell = path.back();

        NodeId endNode;

        if (const auto endNodeId = _graph.findNode(path.back())) {
            endNode = endNodeId.value();
        } else {
            endNode = getNodeId(oldGraph, endCell);
            _graph.createNode(endNode, endCell);
        }

        for (auto& way : ways) {
            queue.emplace(endNode, way);
        }

        _graph.connect(startNode, endNode, path);
    }

    _app.events->Publish<messages::MazeGraphPublishEvent>({ _graph });
}

void MazeGraphEngine::publish() const
{
    auto markers = viz::marker::MarkerArrayBuilder();

    markers.add(viz::marker::clear("map"));

    for (const auto& node : _graph.nodes() | views::values) {

        const auto worldPoint = _map.coordToWorld(node.cell());

        markers.add(viz::marker::point(worldPoint, "map"));
        markers.add(viz::marker::text(worldPoint, std::to_string(node.id()), "map"));
    }

    for (const auto& edge : _graph.edges() | views::values) {
        vector<Vector2> worldPath;
        worldPath.resize(edge.path().size());

        ranges::transform(edge.path(), worldPath.begin(), [&](const auto& p) {
            return Vector2(_map.coordToWorld(p));
        });

        markers.add(viz::marker::path(worldPath, "map"));
    }

    _debugPublisher->publish(markers.array);
}

}
