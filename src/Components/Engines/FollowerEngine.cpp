#include "FollowerEngine.hpp"
#include <limits>

using namespace std;

namespace Manhattan::Core {

FollowerEngine::FollowerEngine(const App& app)
    : RosEngine(app, "follower")
{
    _map = app.GetComponent<MappingEngine>();
    _navigator = app.GetComponent<NavigatorEngine>();
    _graphBuilder = app.GetComponent<NavigatorGraphBuilder>();
}

void FollowerEngine::OnEnable() {
    _initialTimer = create_wall_timer(1s, [this]() {
        _initialTimer->cancel();
        _timer = create_wall_timer(100ms, [this] { Update(); });
    });
}

void FollowerEngine::OnDisable() {
    _timer.reset();
    _initialTimer.reset();
}

void FollowerEngine::Update() {
    FollowCorridor();
}

void FollowerEngine::FollowCorridor() {
    if (!_navigator->IsInDestination())
        return;

    _graphBuilder->BuildGraph(6.0f);
    _graphBuilder->PublishMarkers();

    const auto& graphNodes = _graphBuilder->GetNodes();
    if (graphNodes.empty()) return;

    auto pose = _map->CurrentPose();
    Vector2 poseForward(cos(pose.rotation), sin(pose.rotation));

    if (_currentNode != nullptr) {
        std::shared_ptr<NavigatorNode> sync = nullptr;
        float minSync = 0.5f;
        for (const auto& node : graphNodes) {
            float d = Vector2::Distance(_currentNode->worldPosition, node->worldPosition);
            if (d < minSync) {
                minSync = d;
                sync = node;
            }
        }
        _currentNode = sync;
    }

    if (_currentNode == nullptr) {
        float minDist = std::numeric_limits<float>::max();
        for (const auto& node : graphNodes) {
            float d = Vector2::Distance(pose.position, node->worldPosition);
            if (d < minDist) {
                minDist = d;
                _currentNode = node;
            }
        }

        if (_currentNode != nullptr) {
            auto* cell = _map->GetCell(_currentNode->worldPosition);
            if (cell) _navigator->SetDestination(cell);
        }
        return;
    }

    if (_currentNode->connections.empty()) {
        _currentNode = nullptr;
        return;
    }

    Vector2 referenceDir = poseForward;
    if (_currentEdge != nullptr) {
        Vector2 p1;
        if (_currentEdge->path.size() > 2) {
            p1 = _currentEdge->path[_currentEdge->path.size() - 3];
        } else {
            p1 = (_currentEdge->from != nullptr) ? _currentEdge->from->worldPosition : pose.position;
        }
        Vector2 p2 = _currentNode->worldPosition;
        referenceDir = (p2 - p1).Normalized();
    }

    std::shared_ptr<Edge> bestEdge = nullptr;
    float bestScore = -std::numeric_limits<float>::max();

    for (const auto& edge : _currentNode->connections) {
        if (_currentEdge != nullptr && _currentEdge->from != nullptr) {
            if (Vector2::Distance(edge->to->worldPosition, _currentEdge->from->worldPosition) < 0.1f) {
                if (_currentNode->connections.size() > 1) continue;
            }
        }

        size_t lookAheadIdx = std::min((size_t)4, edge->path.size() > 0 ? edge->path.size() - 1 : 0);
        Vector2 edgeStartIdx = (edge->path.size() > lookAheadIdx) ? edge->path[lookAheadIdx] : edge->to->worldPosition;

        Vector2 edgeDir = (edgeStartIdx - _currentNode->worldPosition).Normalized();
        float dotProduct = Vector2::Dot(referenceDir, edgeDir);

        if (dotProduct > bestScore) {
            bestScore = dotProduct;
            bestEdge = edge;
        }
    }

    if (bestEdge != nullptr) {
        _currentEdge = bestEdge;
        _currentNode = bestEdge->to;

        std::vector<GridCell*> pathCells;
        for (const auto& wp : bestEdge->path) {
            auto* cell = _map->GetCell(wp);
            if (cell != nullptr) {
                if (pathCells.empty() || pathCells.back() != cell) {
                    pathCells.push_back(cell);
                }
            }
        }

        auto* targetCell = _map->GetCell(bestEdge->to->worldPosition);
        if (targetCell != nullptr) {
            if (pathCells.empty() || pathCells.back() != targetCell) {
                pathCells.push_back(targetCell);
            }
        }

        if (pathCells.size() > 1) {
            _navigator->SetPath(pathCells);
        } else if (targetCell != nullptr) {
            _navigator->SetDestination(targetCell);
        }
    } else {
        _currentNode = nullptr;
    }
}

} // namespace Manhattan::Core