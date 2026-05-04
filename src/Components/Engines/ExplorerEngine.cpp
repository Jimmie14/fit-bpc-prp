#include "ExplorerEngine.hpp"
#include <algorithm>
#include <stdexcept>

using namespace std;

namespace Manhattan::Core {
ExplorerEngine::ExplorerEngine(const App& app)
    : RosEngine(app, "explorer")
{
    _slamController = app.GetComponent<MappingEngine>();
    _navigatorController = app.GetComponent<NavigatorEngine>();
}

void ExplorerEngine::OnEnable()
{
    _startCell = _slamController->GetCell(_slamController->CurrentPose().position);
    _state = ExplorerState::Exploring;

    _timer = create_wall_timer(100ms, [this] { Update(); });
}

void ExplorerEngine::OnDisable()
{
    _timer.reset();
}

GridCell* ExplorerEngine::Explore(GridCell* startCell) const
{
    std::map<GridCell*, double> distances;
    std::map<GridCell*, GridCell*> previous;
    std::vector<GridCell*> queue;

    distances[startCell] = 0.0;
    queue.push_back(startCell);
    GridCell* frontier = nullptr;

    while (!queue.empty()) {
        ranges::sort(queue, [&distances](GridCell* a, GridCell* b) { return distances[a] < distances[b]; });

        GridCell* current = queue.front();
        queue.erase(queue.begin());

        bool isBorder = false;

        for (GridCell* neighbor : _slamController->GetNeighbors(current)) {
            double alt = distances[current] + neighbor->GetCost();

            auto distIt = distances.find(neighbor);
            if (distIt != distances.end() && alt >= distIt->second)
                continue;

            distances[neighbor] = alt;
            previous[neighbor] = current;

            if (ranges::find(queue, neighbor) != queue.end())
                continue;

            if (neighbor->IsUnknown()) {
                isBorder = true;
                continue;
            }

            if (neighbor->IsOccupied())
                continue;

            queue.push_back(neighbor);
        }

        if (!isBorder)
            continue;

        frontier = current;
        break;
    }

    return frontier;
}

void ExplorerEngine::Update()
{
    if (!_navigatorController->IsInDestination())
        return;

    switch (_state) {
    case ExplorerState::Idle:
        break;

    case ExplorerState::Exploring: {
        GridCell* startCell = _slamController->GetCell(_slamController->CurrentPose().position);
        if (startCell == nullptr)
            return;

        auto target = Explore(startCell);
        if (target != nullptr) {
            _navigatorController->SetDestination(target);
            break;
        }

        // _state = ExplorerState::Returning;
        // _navigatorController->SetPath(_navigatorController->CalculatePath(_startCell));

        break;
    }
    case ExplorerState::Returning:
        _state = ExplorerState::Idle;
        break;

    default:
        throw std::out_of_range("Invalid ExplorerState");
    }
}
} // namespace Manhattan::Core
