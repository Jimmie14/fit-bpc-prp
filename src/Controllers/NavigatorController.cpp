#include "NavigatorController.hpp"

using namespace std;

namespace Manhattan::Core {
    NavigatorController::NavigatorController(const App &app) : BaseController(app) {
        _motor = app.GetController<MotorController>();
        _slam = app.GetController<SlamController>();

        _pathPublisher = _node->create_publisher<nav_msgs::msg::Path>("~/desired_path", 10);
    }

    void NavigatorController::SetPath(std::vector<GridCell> path) {
        _path = path;

        nav_msgs::msg::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = _node->now();


    }

    std::vector<GridCell> NavigatorController::CalculatePath(GridCell* destination) const {
        std::unordered_map<GridCell*, GridCell*> previous;
        std::vector<GridCell*> queue;

        auto startCell = _slam->GetCell(_slam->CurrentPose().Position);
        if (startCell == nullptr) return { };

        queue.push_back(startCell);

        while (!queue.empty())
        {
            auto current = queue.front();
            queue.erase(queue.begin());

            if (current.GetGridPosition() == destination.GetGridPosition()) break;

            for (GridCell* neighbor : _slam->GetNeighbors(current))
            {
                if (previous.contains(neighbor)) continue;
                previous.emplace(neighbor, current);

                if (ranges::find(queue, neighbor) != queue.end()) continue;
                if (neighbor->IsOccupied()) continue;

                queue.push_back(neighbor);
            }
        }

        std::vector<GridCell> path;
        GridCell current = destination;

        while (current != *startCell)
        {
            path.push_back(current);

            auto it = previous.find(current);
            if (it == previous.end())
                break;

            current = it->second;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    void NavigatorController::ClearPath() {
        _path.clear();
    }
}

