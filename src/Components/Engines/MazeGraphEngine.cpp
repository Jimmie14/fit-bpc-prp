#include "Components/MazeGraphEngine.hpp"

#include "App.hpp"
#include "Components/MapThinningUnit.hpp"
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

static vector<Vector2Int> getNeighbours(const Grid<bool>& skeleton, const Vector2Int& cell)
{
    vector<Vector2Int> result;

    for (auto dir : Vector2Int::EightDirections()) {
        const auto neighbour = cell + dir;

        if (!skeleton.inBounds(neighbour.x, neighbour.y)) continue;
        if (!skeleton[neighbour]) continue;

        result.push_back(neighbour);
    }

    return result;
}

static bool isCrossroad(const Grid<bool>& skeleton, const Vector2Int& cell)
{
    vector frontiers = { cell };
    vector<Vector2Int> nextFrontiers;

    set<Vector2Int> visited;

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

struct Way {
    vector<Vector2Int> path;
    bool foundEnd;
};

static vector<Vector2Int> extractCommonPath(vector<Way>& ways)
{
    vector<Vector2Int> result;

    if (ways.empty()) return result;

    for (const auto& p : ways) {
        if (p.path.empty()) return result;
    }

    auto maxCommonLength = ways[0].path.size();
    for (const auto& way : ways | std::views::drop(1)) {
        if (way.path.size() > maxCommonLength) continue;

        maxCommonLength = way.path.size();
    }

    maxCommonLength--;

    int commonLength = 0;

    for (auto i = 0; i < maxCommonLength; i++) {
        const Vector2Int& candidate = ways[0].path[i];

        const auto allMatch = std::ranges::all_of(ways | std::views::drop(1), [&](const auto& way) { return way.path[i] == candidate; });
        if (!allMatch) break;

        commonLength++;
    }

    if (commonLength == 0) return result;

    result.assign(ways[0].path.begin(), ways[0].path.begin() + commonLength);

    for (auto& way : ways) {
        way.path.erase(way.path.begin(), way.path.begin() + commonLength);
    }

    return result;
}

static vector<Vector2Int> followPath(const Grid<bool>& skeleton, set<Vector2Int>& visited, const Vector2Int& start)
{
    vector<Vector2Int> result;
    vector<Way> ways;
    vector<Way> newWays;

    auto lookaheadVisited = visited;

    ways.push_back(Way { .path = { start }, .foundEnd = false });

    while (true) {
        for (auto& way : ways) {
            const auto frontier = way.path.back();
            auto foundEnd = true;

            for (auto neighbour : getNeighbours(skeleton, frontier)) {
                if (lookaheadVisited.contains(neighbour)) continue;

                lookaheadVisited.insert(neighbour);

                if (foundEnd) {
                    way.path.push_back(neighbour);

                    foundEnd = false;
                } else {
                    auto newWay = way;
                    newWay.path.push_back(neighbour);

                    newWays.push_back(newWay);
                }
            }

            way.foundEnd = foundEnd;
        }

        ways.insert(ways.end(), newWays.begin(), newWays.end());
        newWays.clear();

        if (const auto commonWay = extractCommonPath(ways); !commonWay.empty()) {
            result.insert(result.end(), commonWay.begin(), commonWay.end());
            visited.insert(commonWay.begin(), commonWay.end());
        }

        erase_if(ways, [&](const auto& w) { return w.path.empty() && w.foundEnd; });
        if (ways.empty()) break;

        const auto foundCrossroad = std::ranges::any_of(ways, [](const auto& w) { return w.path.size() > 3; });
        if (foundCrossroad) break;

        erase_if(ways, [](const auto& w) { return w.foundEnd; });
        if (ways.empty()) break;
    }

    return result;
}

vector<Vector2Int> MazeGraphEngine::pathToClosestNode(const Vector2Int& start, const int radius)
{
    vector<Way> ways;
    vector<Way> newWays;
    set<Vector2Int> visited;

    ways.push_back(Way { .path = { start }, .foundEnd = false });

    while (true) {
        for (auto& way : ways) {
            const auto frontier = way.path.back();
            auto foundEnd = true;

            for (auto neighbour : getNeighbours(_skeleton, frontier)) {
                if (visited.contains(neighbour)) continue;

                const auto it = _nodes.find(frontier);
                if (it != _nodes.end()) {
                    return way.path;
                }

                visited.insert(neighbour);

                if (foundEnd) {
                    way.path.push_back(neighbour);

                    foundEnd = false;
                } else {
                    auto newWay = way;
                    newWay.path.push_back(neighbour);

                    newWays.push_back(newWay);
                }
            }

            way.foundEnd = foundEnd;
        }

        ways.insert(ways.end(), newWays.begin(), newWays.end());
        newWays.clear();

        const auto isOnePathTooLong = std::ranges::any_of(ways, [&](const auto& w) { return w.path.size() >= radius; });
        if (isOnePathTooLong) break;

        erase_if(ways, [](const auto& w) { return w.foundEnd; });
        if (ways.empty()) break;
    }

    return { };
}

void MazeGraphEngine::createDeadEnds()
{
    for (int i = 0; i < _skeleton.size(); i++) {
        if (!_skeleton[i]) continue;

        const auto cell = Vector2Int(_skeleton.indexToCoord(i));

        const auto neighbourCount = getNeighbours(_skeleton, cell).size();
        if (neighbourCount != 1) continue;

        _nodes[cell] = Node { cell };
    }
}

void MazeGraphEngine::update()
{
    if (_skeleton.empty()) return;

    _nodes.clear();
    _paths.clear();

    createDeadEnds();

    set<Vector2Int> visited;

    for (auto& cell : _nodes | views::keys) {
        auto path = followPath(_skeleton, visited, cell);
        if (path.size() < 2) continue;

        _paths.push_back(path);


        // auto endCell = path.back();
        //
        // const auto toClosest = pathToClosestNode(endCell, 3);
        // path.insert(path.end(), toClosest.begin(), toClosest.end());
        //
        //
        // endCell = path.back();
        // const auto end = _nodes.find(endCell);
        // if (end == _nodes.end()) {
        //     _nodes[endCell] = Node { endCell };
        // }
    }

    // const auto start = findFirstCell(_skeleton);
    // if (!start.has_value()) return;


}

void MazeGraphEngine::publish() const
{
    auto markers = viz::marker::MarkerArrayBuilder();

    markers.add(viz::marker::clear("map"));

    for (const auto cell : _nodes | views::keys) {

        const auto worldPoint = _map.coordToWorld(cell);

        markers.add(viz::marker::point(worldPoint, "map"));
    }

    for (const auto& path : _paths) {

        vector<Vector2> worldPath;
        worldPath.resize(path.size());

        ranges::transform(path, worldPath.begin(), [&](const auto& p) {
            return Vector2(_map.coordToWorld(p));
        });

        markers.add(viz::marker::path(worldPath, "map"));
    }


    _debugPublisher->publish(markers.array);
}

}
