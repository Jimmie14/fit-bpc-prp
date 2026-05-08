#pragma once
#include "Math/Pose.hpp"

namespace Manhattan::messages {

using namespace Manhattan::math;

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

struct MotorCommandEvent {
    Twist twist;
};

}