#pragma once

#include "Common/Attributes.hpp"
#include "Math/LinearPath.hpp"
#include "Math/Vec2i.hpp"
#include "Math/Vector2.hpp"

#include <map>
#include <queue>
#include <ranges>

namespace Manhattan::nav {

using namespace std;
using namespace math;
using namespace common;

using NodeId = unsigned int;
using EdgeId = pair<NodeId, NodeId>;

struct GraphPosition {
    GraphPosition() = default;

    explicit GraphPosition(const Vector2i& cell, const NodeId start, const NodeId end, const int costToStart, const int costToEnd)
        : from(start)
        , to(end)
        , cell(cell)
        , costToStart(costToStart)
        , costToEnd(costToEnd)
    {

    }

    NodeId from = 0;
    NodeId to = 0;

    Vector2i cell = vec2i::zero;

    int costToStart = 0;
    int costToEnd = 0;
};


class Node {
public:
    Node() = default;

    Node(const NodeId id, const Vector2i& cell)
        : _id(id)
        , _cell(cell)
    {

    }

    Attributes attributes;

    NodeId id() const { return _id; }
    Vector2i cell() const { return _cell; }

private:
    NodeId _id = 0;
    Vector2i _cell = vec2i::zero;
};

class Edge {
public:

    Edge() = default;

    Edge(const NodeId from, const NodeId to, const vector<Vector2i>& path)
        : _from(from)
        , _to(to)
        , _path(path)
    {

    }

    Attributes attributes;

    NodeId from() const { return _from; }
    NodeId to() const { return _to; }
    const vector<Vector2i>& path() const { return _path; }

private:
    NodeId _from = 0;
    NodeId _to = 0;
    vector<Vector2i> _path;
};

class MazeGraph {
public:
    MazeGraph() = default;

    void createNode(const NodeId id, const Vector2i& cell)
    {
        const auto node = Node(id, cell);

        _nodes[id] = node;
        _nodeMap[cell] = id;
    }

    optional<NodeId> findNode(const Vector2i& cell)
    {
        if (!_nodeMap.contains(cell)) return nullopt;

        return _nodeMap[cell];
    }

    optional<NodeId> findClosestNode(const Vector2i& cell, const double radius) const
    {
        if (_nodes.empty()) return nullopt;

        auto minSq = radius * radius;
        optional<NodeId> result;

        for (auto& node : _nodes | views::values) {
            const auto distance = (Vector2(node.cell()) - Vector2(cell)).sqrMagnitude();
            if (distance > minSq) continue;

            minSq = distance;
            result = node.id();
        }

        return result;
    }

    optional<GraphPosition> findClosestPosition(const Vector2i& cell, const double radius) const
    {
        const auto point = Vector2(cell);

        optional<GraphPosition> result;

        auto bestDistanceSq = radius * radius;
        vector<Vector2> buffer;

        for (auto& edge : _edges | views::values) {
            const auto& path = edge.path();

            buffer.resize(path.size());

            ranges::transform(path, buffer.begin(), [&](const auto& p) {
                return Vector2(p);
            });

            auto lPath = LinearPath();
            lPath.init(buffer);

            const auto segmentResult = lPath.findClosestPoint(point);
            if (segmentResult.distanceToPathSq > bestDistanceSq) continue;

            bestDistanceSq = segmentResult.distanceToPathSq;

            result = GraphPosition(Vector2i::round(segmentResult.position), edge.from(), edge.to(),
                lPath.getTotalLength() - segmentResult.distanceAlongPath,
                segmentResult.distanceAlongPath);
        }

        return result;
    }

    bool containsNode(const Vector2i& cell) const
    {
        return _nodeMap.contains(cell);
    }

    bool containsNode(const NodeId id) const
    {
        return _nodes.contains(id);
    }

    void connect(NodeId from, NodeId to, const vector<Vector2i>& path)
    {
        _edges[{from, to}] = Edge(from, to, path);
        _toEdges[from].push_back(to);
        _toEdges[to].push_back(from);
    }

    bool containsEdge(NodeId from, NodeId to) const
    {
        return _edges.contains({ from, to }) || _edges.contains({ to, from });
    }

    const Edge& getEdge(NodeId from, NodeId to) const
    {
        if (_edges.contains({ from, to })) {
            return _edges.at({ from, to });
        }

        return _edges.at({ to, from });
    }

    Edge& getEdge(NodeId from, NodeId current, float turn);

    optional<NodeId> followEdge(NodeId from, NodeId current, float turn) const
    {
        if (!_edges.contains({ from, current })) {
            return nullopt;
        }

        const auto neighbours = getNeighbours(current);
        if (neighbours.empty()) return nullopt;

        if (neighbours.size() == 1) return neighbours.front();


        // todo: finish implementation
        // const auto p1 = getEdge(from, current).path().

        return nullopt;
    }

    bool existEdge(NodeId from, NodeId to) const
    {
        return _edges.contains({ from, to }) || _edges.contains({ to, from });
    }

    vector<NodeId> getNeighbours(const NodeId from) const
    {
        const auto it = _toEdges.find(from);
        if (it == _toEdges.end()) return {};

        return it->second;
    }

    const unordered_map<NodeId, Node>& nodes() const
    {
        return _nodes;
    }

    const map<pair<NodeId, NodeId>, Edge>& edges() const
    {
        return _edges;
    }

    optional<vector<Vector2i>> calculatePath(const GraphPosition& start, const GraphPosition& end) const
    {
        if (start.from == end.from && start.to == end.to) {
            const auto path = getEdge(end.from, end.to).path();
            vector<Vector2i> result;

            if (start.costToStart < end.costToStart) {
                auto view = path
                    | views::drop_while([&](const auto& p) { return p != end.cell; })
                    | views::take_while([&](const auto& p) { return p != start.cell; });

                for (auto p : view) {
                    result.push_back(p);
                }

                return result;
            }

            auto view = path
                | views::drop_while([&](const auto& p) { return p != start.cell; })
                | views::take_while([&](const auto& p) { return p != end.cell; });

            for (auto p : view) {
                result.push_back(p);
            }

            return result;
        }

        struct Candidate {
            NodeId from;
            NodeId to;
            optional<vector<Vector2i>> path;
            int cost;
        };

        auto makeCandidate = [&](NodeId a, NodeId b, const int baseCost) -> Candidate
        {
            auto p = calculatePath(a, b);
            if (!p.has_value()) return {a, b, std::nullopt, INT_MAX};

            const int cost = baseCost + static_cast<int>(p.value().size());

            return { a, b, p, cost };
        };

        vector<Candidate> candidates;

        candidates.push_back(makeCandidate(start.from, end.from,start.costToStart + end.costToStart));
        candidates.push_back(makeCandidate(start.from, end.to, start.costToStart + end.costToEnd));
        candidates.push_back(makeCandidate(start.to, end.from, start.costToEnd + end.costToStart));
        candidates.push_back(makeCandidate(start.to, end.to, start.costToEnd + end.costToEnd));

        const Candidate* best = nullptr;

        for (const auto& c : candidates)
        {
            if (!c.path.has_value()) continue;

            if (!best || c.cost < best->cost) best = &c;
        }

        if (!best) return std::nullopt;

        auto path = best->path.value();

        auto startPath = getPathTo(start, best->from);
        path.insert(path.begin(), startPath.begin(), startPath.end());


        auto endPath = getPathTo(end, best->to);
        ranges::reverse(endPath);
        path.insert(path.end(), endPath.begin(), endPath.end());


        return path;
    }

    optional<vector<Vector2i>> calculatePath(const NodeId start, const NodeId end) const
    {
        std::priority_queue<
            std::pair<int, NodeId>,
            std::vector<std::pair<int, NodeId>>,
            std::greater<>> open;

        unordered_map<NodeId, NodeId> previous;

        open.emplace(0, start);

        while (true) {
            if (open.empty()) return nullopt;

            const auto [cost, current] = open.top(); open.pop();

            if (current == end) break;

            const auto nextNodes = _toEdges.find(current);
            if (nextNodes == _toEdges.end()) continue;

            for (auto& next : nextNodes->second) {
                const auto edge = getEdge(current, next);

                const auto nextCost = cost + edge.path().size();

                if (previous.contains(next)) continue;

                previous[next] = current;
                open.emplace(nextCost, next);
            }
        }

        vector<Vector2i> path;

        for (auto current = end; current != start; ) {
            auto next = previous[current];

            const auto& edge = getEdge(current, next);
            if (edge.from() == current) {
                path.insert(path.end(), edge.path().begin(), edge.path().end());
            } else {
                path.insert(path.end(), edge.path().rbegin(), edge.path().rend());
            }

            current = next;
        }

        ranges::reverse(path);

        return path;
    }

    void update(MazeGraph other)
    {
        const auto oldNodes = _nodes;
        const auto oldEdges = _edges;

        _nodes = std::move(other._nodes);
        _nodeMap = std::move(other._nodeMap);

        _edges = std::move(other._edges);
        _toEdges = std::move(other._toEdges);

        for (auto& [id, node] : oldNodes) {
            auto it = _nodes.find(id);
            if (it == _nodes.end()) continue;

            it->second.attributes = node.attributes;
        }

        for (auto& [key, edge] : oldEdges) {
            auto it = _edges.find(key);
            if (it == _edges.end()) continue;

            it->second.attributes = edge.attributes;
        }
    }

private:
    std::unordered_map<NodeId, Node> _nodes;
    std::unordered_map<Vector2i, NodeId, Vector2iHash> _nodeMap;

    std::map<EdgeId, Edge> _edges;
    std::unordered_map<NodeId, vector<NodeId>> _toEdges;

    vector<Vector2i> getPathTo(const GraphPosition& position, const NodeId end) const
    {
        vector<Vector2i> result;

        const auto edge = getEdge(position.from, position.to);

        if (position.to == end) {
            auto view = edge.path()
                | views::drop_while([&](const auto& p) { return p != position.cell; });

            for (auto p : view) {
                result.push_back(p);
            }

            return result;
        }

        auto view = edge.path()
            | views::take_while([&](const auto& p) { return p != position.cell; })
            | views::reverse;

        for (auto p : view) {
            result.push_back(p);
        }

        return result;
    }
};




}