#pragma once

#include "RosComponent.hpp"

namespace Manhattan::Core {
class RosEngine : public RosComponent {
public:
    explicit RosEngine(const App& app, const string& name)
        : RosComponent(app, name + "_engine")
    {
    }
};
}
