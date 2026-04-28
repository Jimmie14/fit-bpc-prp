#pragma once

#include <vector>

using namespace std;

namespace Manhattan::Grid {

template <typename T>
class Grid {
public:
    const int width;
    const int height;
    const double resolution;

    Grid(const int width, const int height, const double resolution, const T& defaultValue = T())
        : width(width)
        , height(height)
        , resolution(resolution)
    {
        _data.reserve(width * height);

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                _data.push_back(defaultValue);
            }
        }
    }

    T& operator()(const int x, const int y)
    {
        return _data[index(x, y)];
    }

    const T& operator()(const int x, const int y) const
    {
        return _data[index(x, y)];
    }

    T& get(const int x, const int y) const
    {
        return _data[index(x, y)];
    }

    class RowProxy {
    public:
        RowProxy(const Grid& grid, const int x)
            : _parent(grid)
            , _x(x)
        {
        }

        T& operator[](const int y) {
            return _parent(_x, y);
        }
    private:
        Grid& _parent;
        const int _x;
    };

    class ConstRowProxy {
    public:
        ConstRowProxy(Grid& grid, const int x)
            : _parent(grid)
            , _x(x)
        {
        }

        const T& operator[](const int y) const {
            return _parent(_x, y);
        }
    private:
        const Grid& _parent;
        const int _x;
    };


    RowProxy operator[](std::size_t x) {
        return RowProxy(*this, x);
    }

    ConstRowProxy operator[](std::size_t x) const {
        return ConstRowProxy(*this, x);
    }
private:
    vector<T> _data;

    int index(const int x, const int y) const {
        return y * width + x;
    }
};

}