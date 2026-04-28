#pragma once

#include "RosComponent.hpp"

namespace Manhattan::Core {
class RosUnit : public RosComponent {
public:
    explicit RosUnit(const App& app, const string& name)
        : RosComponent(app, name + "_unit")
    {
    }
};
}
