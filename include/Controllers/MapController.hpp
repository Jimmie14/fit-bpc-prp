#pragma once

#include "BaseController.h"
#include "LidarController.hpp"

namespace Manhattan::Core
{
    class MapController final : public BaseController
    {
        void Update(const std::vector<Point> &scan);

    public:
        explicit MapController(const App& app);
    };
}
