#pragma once

#include <tf2/LinearMath/Vector3.hpp>

namespace Manhattan::nav {

using namespace tf2;

class GridMap {
public:
    GridMap(const unsigned int width, const unsigned int height, const float resolution)
        : _width(width)
        , _height(height)
        , _resolution(resolution)
    {

    }

    [[nodiscard]] unsigned int width() const
    {
        return _width;
    }

    [[nodiscard]] unsigned int height() const
    {
        return _height;
    }

    [[nodiscard]] float resolution() const
    {
        return _resolution;
    }

    [[nodiscard]] std::pair<int, int> worldToCoord(const Vector3& position) const
    {
        int x = std::floor(position.x() / _resolution + static_cast<float>(_width) * 0.5f);
        int y = std::floor(position.y() / _resolution + static_cast<float>(_height) * 0.5f);
        return std::pair<int, int>(x, y);
    }

    [[nodiscard]] Vector3 worldToLocal(const Vector3& position) const
    {
        return {
            std::floor(position.x() / _resolution + static_cast<float>(_width) * 0.5f),
            std::floor(position.y() / _resolution + static_cast<float>(_height) * 0.5f),
            0.0f
        };
    }

    [[nodiscard]] Vector3 coordToWorld(const std::pair<int, int>& coord) const
    {
        return coordToWorld(coord.first, coord.second);
    }

    [[nodiscard]] Vector3 coordToWorld(const Math::Vector2Int& coord) const
    {
        return coordToWorld(coord.x, coord.y);
    }

    [[nodiscard]] Vector3 coordToWorld(const int x, const int y) const
    {
        return {
            (static_cast<float>(x) - static_cast<float>(_width) * 0.5f) * _resolution + _resolution * 0.5f,
            (static_cast<float>(y) - static_cast<float>(_height) * 0.5f) * _resolution + _resolution * 0.5f,
            0.0f
        };
    }

    [[nodiscard]] Vector3 clamp(const Vector3& position) const
    {
        return coordToWorld(worldToCoord(position));
    }

private:
    unsigned int _width;
    unsigned int _height;
    float _resolution;
};
}

