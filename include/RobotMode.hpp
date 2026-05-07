#pragma once

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