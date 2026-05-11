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

    explicit GraphPosition(const Vector2i& cell, const NodeId from, const NodeId to)
        : from(from)
        , to(to)
        , cell(cell)
    {

    }

    NodeId from = 0;
    NodeId to = 0;

    Vector2i cell = vec2i::zero;
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
            for (auto& p : path) {
                buffer.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y));
            }

            auto lPath = LinearPath();
            lPath.init(buffer);

            const auto segmentResult = lPath.findClosestPoint(point);
            if (segmentResult.distanceToPathSq > bestDistanceSq) continue;

            bestDistanceSq = segmentResult.distanceToPathSq;

            result = GraphPosition(Vector2i::round(segmentResult.position), edge.from(), edge.to());
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

    const Edge& getEdge(NodeId from, NodeId to) const
    {
        if (_edges.contains({ from, to })) {
            return _edges.at({ from, to });
        }

        return _edges.at({ to, from });
    }

    Edge& getEdge(NodeId from, NodeId current, float turn);

    const Edge& getEdge(NodeId from, NodeId current, float turn) const;

    const unordered_map<NodeId, Node>& nodes() const
    {
        return _nodes;
    }

    const map<pair<NodeId, NodeId>, Edge>& edges() const
    {
        return _edges;
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

private:
    std::unordered_map<NodeId, Node> _nodes;
    std::unordered_map<Vector2i, NodeId, Vector2iHash> _nodeMap;

    std::map<EdgeId, Edge> _edges;
    std::unordered_map<NodeId, vector<NodeId>> _toEdges;
};




}