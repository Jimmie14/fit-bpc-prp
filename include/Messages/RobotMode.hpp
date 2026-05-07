#pragma once
#include "Math/Pose.hpp"

namespace Manhattan::Messages {

struct RobotMode {
    bool reverse = false;
    bool motorOff = false;
};

struct RobotModeChangeEvent {
    RobotMode oldMode;
    RobotMode newMode;
};


struct RobotResetEvent {

};

struct RobotEnvironmentChangeEvent {

};

}