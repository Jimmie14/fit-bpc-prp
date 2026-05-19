#pragma once

#include "Math/Pose.hpp"
#include "Nav/MazeGraph.hpp"
#include <tf2/LinearMath/Vector3.hpp>
#include <vector>

namespace Manhattan::messages {

using namespace Manhattan::math;

struct RobotFollowPathEvent {
    std::vector<Vector3> path;
};

struct RobotPoseEvent {
    Pose pose = Pose::zero();
    Twist twist = Twist::zero();
};

struct MazeGraphPublishEvent {
    nav::MazeGraph graph;
};

}
