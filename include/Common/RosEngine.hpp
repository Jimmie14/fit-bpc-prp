#pragma once

#include "RosConnector.hpp"

namespace Manhattan::Core {
class RosEngine : public RosConnector {
public:
    explicit RosEngine(const App& app)
        : RosConnector(app)
    {
    }
};
}

