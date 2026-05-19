#pragma once

#include "App.hpp"
#include "Common/RosUnit.hpp"
#include "Messages/Nav.hpp"
#include "Messages/RobotMode.hpp"

namespace Manhattan::core {

class DebugUnit : public RosUnit
{
public:
    explicit DebugUnit(const App& app) : RosUnit(app, "debug")
    {
        app.events->Subscribe<messages::RobotPoseEvent>([&](const auto& event) {
            std::cout << "twist  : " << event.twist << std::endl;
        });
    }
protected:
    void OnEnable() override
    {
        _app.events->Publish(MotorCommandEvent { Twist(0, M_PI / 8) });
    }

    void OnDisable() override
    {

    }

private:


};

}

