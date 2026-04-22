#pragma once

#include "RosConnector.hpp"

namespace Manhattan::Core {
class RosDeviceDriver : public RosConnector {
public:
    explicit RosDeviceDriver(const App& app)
        : RosConnector(app)
    {
    }
};
}
