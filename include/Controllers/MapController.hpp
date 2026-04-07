#pragma once

#include "BaseController.h"
#include "PoseMatcher.hpp"

namespace Manhattan::Core
{
    class MapController final : public BaseController
    {
        Pose _currentPose;
        OccupancyGrid _grid;
        PoseMatcher _matcher;

        void Update(const std::vector<Point> &scan);

        void ResetGridIfNeeded();

        void MapScan();

        Point LocalToWorld(const Point &point);

        std::vector<GridCell&> GetNeighbors(const GridCell &point);

        GridCell& GetCell(const Point &worldPos);

        double GetFreeDistanceAhead(Point pos, Point forward, double maxDistance = 5);

    public:
        explicit MapController(const App& app);
    };
}
