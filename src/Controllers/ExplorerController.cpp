#include "ExplorerController.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace std;

namespace Manhattan::Core {
ExplorerController::ExplorerController(const App& app)
    : BaseController(app)
{
    _slamController = app.GetController<SlamController>();
    _navigatorController = app.GetController<NavigatorController>();
}

void ExplorerController::OnEnable()
{
    _startCell = _slamController->GetCell(_slamController->CurrentPose().position);
    _state = ExplorerState::Exploring;

    _timer = _node->create_wall_timer(100ms, [this] { Update(); });
}

void ExplorerController::OnDisable()
{
    _timer.reset();
}

std::queue<GridCell*> ExplorerController::Explore(GridCell* startCell) const
{
    std::map<GridCell*, double> distances;
    std::map<GridCell*, GridCell*> previous;
    std::vector<GridCell*> queue;

    distances[startCell] = 0.0;
    queue.push_back(startCell);
    GridCell* frontier = nullptr;

    while (!queue.empty()) {
        // Sort queue by distance
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

    std::vector<GridCell*> pathList;
    GridCell* curr = frontier;

    while (curr != nullptr && curr != startCell) {
        pathList.push_back(curr);
        auto prevIt = previous.find(curr);
        if (prevIt != previous.end()) {
            curr = prevIt->second;
        } else {
            curr = nullptr;
        }
    }
    ranges::reverse(pathList);

    std::queue<GridCell*> pathQueue;
    long i = 0;
    for (auto* cell : pathList) {
        pathQueue.push(cell);
        i++;

        if (i >= pathList.size() - 5)
            break;
    }

    return pathQueue;
}

void ExplorerController::Update()
{
    if (_navigatorController->HasPath())
        return;

    switch (_state) {
    case ExplorerState::Idle:
        break;

    case ExplorerState::Exploring: {
        GridCell* startCell = _slamController->GetCell(_slamController->CurrentPose().position);
        if (startCell == nullptr)
            return;

        std::queue<GridCell*> path = Explore(startCell);
        if (!path.empty()) {
            _navigatorController->SetPath(path);
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
