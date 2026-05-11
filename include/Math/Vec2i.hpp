#pragma once

#include "Math/Vector2.hpp"

using tf2::Vector3;

namespace Manhattan::math::vec2i {
const Vector2i zero(0, 0);
const Vector2i one(1, 1);

const Vector2i right(0, 1);
const Vector2i left(0, -1);

const Vector2i forward(1, 0);
const Vector2i backward(-1, 0);

}