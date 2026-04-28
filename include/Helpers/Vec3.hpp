#pragma once

#include <tf2/LinearMath/Vector3.hpp>

using tf2::Vector3;

namespace vec3 {
const Vector3 Zero(0.0, 0.0, 0.0);
const Vector3 One(1.0, 1.0, 1.0);

const Vector3 Right(1.0, 0.0, 0.0);
const Vector3 Left(-1.0, 0.0, 0.0);

const Vector3 Up(0.0, 1.0, 0.0);
const Vector3 Down(0.0, -1.0, 0.0);

const Vector3 Forward(0.0, 0.0, 1.0);
const Vector3 Backward(0.0, 0.0, -1.0);

}