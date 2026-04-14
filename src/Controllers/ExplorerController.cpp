#include "ExplorerController.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

using namespace std;

namespace Manhattan::Core
{
    ExplorerController::ExplorerController(const App& app)
        : BaseController(app)
    {
        _slamController = app.GetController<SlamController>();
        _navigatorController = app.GetController<NavigatorController>();
    }

    void ExplorerController::OnEnable()
    {
        _startCell = _slamController->GetCell(_slamController->CurrentPose().position);

        _timer = _node->create_wall_timer(
            100ms,
            [this] { Update(); }
        );
    }

    void ExplorerController::OnDisable()
    {
        _timer.reset();
    }

    GridCell* ExplorerController::Recenter(GridCell* cell) const
    {
        double angle = 0.0;
        double angleStep = M_PI * 0.5;

        std::vector<RayHit> hits;

        for (int i = 0; i < 4; i++)
        {
            Vector2 direction(std::cos(angle), std::sin(angle));
            RayHit rayHit;

            bool hit = _slamController->RayCast(cell->GetWorldPosition(), direction, rayHit, 20.0);

            angle += angleStep;
            if (!hit) continue;

            hits.push_back(rayHit);
        }

        if (hits.empty()) return cell;

        Vector2 acc(0, 0);
        for (const auto& hit : hits) {
            acc = acc + hit.normal;
        }

        Vector2 averageNormal = acc.Normalized();
        Vector2 centered = cell->GetWorldPosition() + averageNormal * 0.5f;

        GridCell* newCell = _slamController->GetCell(centered);
        return newCell != nullptr ? newCell : cell;
    }

    std::queue<GridCell*> ExplorerController::Explore(GridCell* startCell) const
    {
        std::map<GridCell*, double> distances;
        std::map<GridCell*, GridCell*> previous;
        std::vector<GridCell*> queue;

        distances[startCell] = 0.0;
        queue.push_back(startCell);
        GridCell* frontier = nullptr;

        while (!queue.empty())
        {
            // Sort queue by distance
            ranges::sort(queue, [&distances](GridCell* a, GridCell* b) {
                return distances[a] < distances[b];
            });

            GridCell* current = queue.front();
            queue.erase(queue.begin());

            bool isBorder = false;

            for (GridCell* neighbor : _slamController->GetNeighbors(current))
            {
                double alt = distances[current] + neighbor->GetCost();

                auto distIt = distances.find(neighbor);
                if (distIt != distances.end() && alt >= distIt->second) continue;

                distances[neighbor] = alt;
                previous[neighbor] = current;

                if (ranges::find(queue, neighbor) != queue.end()) continue;

                if (neighbor->IsUnknown())
                {
                    isBorder = true;
                    continue;
                }

                if (neighbor->IsOccupied()) continue;

                queue.push_back(neighbor);
            }

            if (!isBorder) continue;

            frontier = current;
            break;
        }

        std::vector<GridCell*> pathList;
        GridCell* curr = frontier;

        while (curr != nullptr && curr != startCell)
        {
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
        for (auto* cell : pathList) {
            pathQueue.push(cell);
        }

        return pathQueue;
    }

    void ExplorerController::Update()
    {
        if (_navigatorController->HasPath()) return;

        switch (_state)
        {
            case ExplorerState::Idle:
                break;

            case ExplorerState::Exploring:
            {
                GridCell* startCell = _slamController->GetCell(_slamController->CurrentPose().position);
                if (startCell == nullptr) return;

                std::queue<GridCell*> path = Explore(startCell);
                if (!path.empty())
                {
                    _navigatorController->SetPath(path);
                    break;
                }

                _state = ExplorerState::Returning;
                _navigatorController->SetPath(_navigatorController->CalculatePath(_startCell));

                break;
            }
            case ExplorerState::Returning:
                _state = ExplorerState::Idle;
                break;

            default:
                throw std::out_of_range("Invalid ExplorerState");
        }
    }
}
