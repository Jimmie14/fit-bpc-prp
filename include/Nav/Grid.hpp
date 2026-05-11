#pragma once

#include "Math/Vector2.hpp"

#include <ranges>
#include <vector>

using namespace std;

namespace Manhattan::nav {

using namespace Manhattan::math;

template <typename T>
class Grid {
public:
    explicit Grid(const unsigned int width, const unsigned int height, const float resolution, const T& defaultValue = T())
        : _width(width)
        , _height(height)
        , _resolution(resolution)
    {
        _data.reserve(width * height);

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                _data.push_back(defaultValue);
            }
        }
    }

    static Grid<T> uninitialized()
    {
        return Grid<T>(0, 0, 0.0f);
    }

    [[nodiscard]] bool empty() const
    {
        return _data.empty();
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

    [[nodiscard]] size_t size() const
    {
        return _data.size();
    }

    [[nodiscard]] ranges::iota_view<unsigned int, unsigned int> columns() const
    {
        return std::views::iota(static_cast<unsigned int>(0), _width);
    }

    [[nodiscard]] ranges::iota_view<unsigned int, unsigned int> rows() const
    {
        return std::views::iota(static_cast<unsigned int>(0), _height);
    }

    T operator[](const int index) const
    {
        return _data[index];
    }

    T operator[](const pair<int, int> coord) const
    {
        return _data[coordToIndex(coord.first, coord.second)];
    }

    T operator[](const Vector2i coord) const
    {
        return _data[coordToIndex(coord.x, coord.y)];
    }

    T& operator()(const int x, const int y)
    {
        return _data[coordToIndex(x, y)];
    }

    T operator()(const int x, const int y) const
    {
        return _data[coordToIndex(x, y)];
    }

    T get(const int x, const int y) const
    {
        return _data[coordToIndex(x, y)];
    }

    void set(const int x, const int y, const T value)
    {
        _data[coordToIndex(x, y)] = value;
    }

    void set(const int index, const T value)
    {
        _data[index] = value;
    }

    void setChecked(const int x, const int y, const T value)
    {
        if (!inBounds(x, y)) return;

        _data[coordToIndex(x, y)] = value;
    }

    [[nodiscard]] bool inBounds(const int x, const int y) const
    {
        return x >= 0 && x < _width && y >= 0 && y < _height;
    }

    [[nodiscard]] int coordToIndex(const int x, const int y) const
    {
        return y * _width + x;
    }

    [[nodiscard]] pair<int, int> indexToCoord(const int index) const
    {
        return { index % _width, index / _width };
    }

private:
    unsigned int _width;
    unsigned int _height;
    float _resolution;
    vector<T> _data;
};

}