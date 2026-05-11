#pragma once

#include <tf2/LinearMath/Vector3.hpp>
#include <vector>

namespace Manhattan::messages {

struct RobotFollowPathEvent {
    std::vector<tf2::Vector3> path;
};

struct RobotPoseEvent {
    core::Pose pose;
    core::Twist twist;
};

}
