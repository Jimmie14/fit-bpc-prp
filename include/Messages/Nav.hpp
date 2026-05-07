#pragma once

#include <tf2/LinearMath/Vector3.hpp>
#include <vector>

namespace Manhattan::Messages {

struct RobotFollowPathEvent {
    std::vector<tf2::Vector3> path;
};

struct RobotPoseEvent {
    Core::Pose pose;
    Core::Twist twist;
};

}
