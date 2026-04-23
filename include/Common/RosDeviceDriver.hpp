#pragma once

#include "RosComponent.hpp"

namespace Manhattan::Core {
class RosDeviceDriver : public RosComponent {
public:
    explicit RosDeviceDriver(const App& app, const string& name)
        : RosComponent(app, name + "_driver")
    {
    }
};
}
