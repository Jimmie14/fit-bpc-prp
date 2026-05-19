#include "Math/Vector2.hpp"

namespace Manhattan::math {

Vector2i Vector2i::round(const Vector2& vector)
{
    return Vector2i(std::round(vector.x), std::round(vector.y));
}

}