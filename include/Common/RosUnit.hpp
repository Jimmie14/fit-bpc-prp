#pragma once

#include "RosComponent.hpp"

namespace Manhattan::core {
class RosUnit : public RosComponent {
public:
    explicit RosUnit(const App& app, const string& name)
        : RosComponent(app, name + "_unit")
    {
    }
};
}
