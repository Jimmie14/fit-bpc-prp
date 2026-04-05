#include "MapController.hpp"
#include "OccupancyGrid.hpp"

#include "App.h"

using namespace std;

namespace Manhattan::Core
{
    MapController::MapController(const App& app) : BaseController(app)
    {
        app.GetController<LidarController>()->SetScanCallback(
        [this](const std::vector<Point>& points) { this->Update(points); }
    );
    }

    void MapController::Update(const std::vector<Point> &points)
    {

    }
}
