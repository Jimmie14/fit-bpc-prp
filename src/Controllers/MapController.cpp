#include "MapController.hpp"
#include "LidarController.hpp"

#include "App.h"

using namespace std;

namespace Manhattan::Core
{
    MapController::MapController(const App& app)
        : BaseController(app),
        _grid(Vector2Int(100, 100), 0.2, 5),
        _matcher(PoseMatcher(_grid, 5))
    {
        app.GetController<LidarController>()->SetScanCallback(
        [this](const std::vector<Point>& points) { this->Update(points); }
        );

        _currentPose = { Point(0, 0), 0 };
    }

    void MapController::Update(const std::vector<Point> &points)
    {

    }
}
