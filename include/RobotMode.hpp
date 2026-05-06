#pragma once

struct RobotMode {
    bool reverse = false;
};

struct RobotModeChangeEvent {
    RobotMode oldMode;
    RobotMode newMode;
};

struct RobotResetEvent {

};