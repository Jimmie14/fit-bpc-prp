#pragma once

#include <assert.h>
#include <ranges>
#include <vector>

using namespace std;

namespace Manhattan::Nav {

template <typename T>
class Grid {
public:
    const int width;
    const int height;
    const size_t size;
    const double resolution;

    const ranges::iota_view<int, int> columns;
    const ranges::iota_view<int, int> rows;

    explicit Grid(const int width, const int height, const double resolution, const T& defaultValue = T())
        : width(width)
        , height(height)
        , size(width * height)
        , resolution(resolution)
        , columns(std::views::iota(0, width))
        , rows(std::views::iota(0, height))
    {
        _data.reserve(size);

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                _data.push_back(defaultValue);
            }
        }
    }

    T operator[](const int index) const
    {
        return _data[index];
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

    int coordToIndex(const int x, const int y) const
    {
        return y * width + x;
    }

    pair<int, int> indexToCoord(const int index) const
    {
        return { index % width, index / width };
    }

private:
    vector<T> _data;
};

}