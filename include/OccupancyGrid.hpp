#pragma once
#include <algorithm>
#include <optional>
#include <queue>
#include <set>

#include "Point.hpp"

namespace Manhattan::Core
{
    class GridCell
    {
        double _probability = 0;
        double _cost = 0;

        Point _worldPosition;
        Vector2Int _gridPosition;

    public:
        GridCell(Vector2Int gridPos, Point worldPos)
        {
            _gridPosition = gridPos;
            _worldPosition = worldPos;
        }

        Vector2Int GetGridPosition() const { return _gridPosition; }

        Point GetWorldPosition() const { return _worldPosition; }

        double Probability() const { return 1.0 - 1.0 / (1.0 + std::exp(_probability)); }

        bool IsFree() const { return Probability() < 0.5f; }

        bool IsOccupied() const { return Probability() >= 0.5f; }

        double GetCost() const
        {
            if (IsOccupied())
                return std::numeric_limits<double>::max();

            return _cost;
        }

        void SetCost(double cost) { _cost = cost; }

        void Add(const double logOdds)
        {
            _probability = std::clamp(_probability + logOdds, -5.0, 20.0);
        }
    };

    class OccupancyGrid
    {
        int _width;
        int _height;

        double _cellSize;

        int _costSteps;
        double _inflationPenalty;

        std::vector<GridCell> _grid;

        const double LogOddsFree = -0.04f;
        const double LogOddsOccupied = 1.85f;

        int GetIndex(int x, int y) const {
            return y * _width + x;
        }

        public:
        OccupancyGrid(const Vector2Int size, const double cellSize, const int costSteps = 5, const double inflationPenalty = 5) : _cellSize(cellSize)
        {
            _width = size.x;
            _height = size.y;

            _costSteps = costSteps;
            _inflationPenalty = inflationPenalty;

            for (int y = 0; y < _height; y++)
            for (int x = 0; x < _width; x++)
            {
                auto cell = GridCell(Vector2Int(x, y), GridToWorld(Vector2Int(x, y)));
                _grid.emplace_back(cell);
            }
        }

        Vector2Int WorldToGrid(const Vector2Int &worldPos) const
        {
            return Vector2Int(
                std::floor(worldPos.x / _cellSize + _width * 0.5),
                std::floor(worldPos.y / _cellSize + _height * 0.5)
            );
        }

        Point GridToWorld(const Vector2Int &gridPos) const
        {
            return Point(
                (gridPos.x - _width * 0.5) * _cellSize + _cellSize * 0.5,
                (gridPos.y - _height * 0.5) * _cellSize + _cellSize * 0.5
            );
        }

        bool InBounds(const int x, const int y) const
        {
            return x >= 0 && x < _width && y >= 0 && y < _height;
        }

        double GetProbability(int x, int y) const
        {
            if (!InBounds(x, y)) return 0.5;

            return _grid[GetIndex(x, y)].Probability();
        }

        void SetFree(const Vector2Int cell, const double dst)
        {
            if (!InBounds(cell.x, cell.y)) return;

            _grid[GetIndex(cell.x, cell.y)].Add(LogOddsFree * std::exp(-dst * 0.5f));
        }

        void SetOccupied(const Vector2Int cell)
        {
            if (!InBounds(cell.x, cell.y)) return;

            _grid[GetIndex(cell.x, cell.y)].Add(LogOddsOccupied);
        }

        std::optional<GridCell> GetCell(const Vector2Int cell)
        {
            if (!InBounds(cell.x, cell.y)) return std::nullopt;

            return _grid[GetIndex(cell.x, cell.y)];
        }

        void RecalculateCosts()
        {
            std::queue<Vector2Int> queue;
            std::set<Vector2Int> visited;

            for (int i = 0; i < _grid.size(); i++)
            {
                auto& cell = _grid[i];
                cell.SetCost(0.0);

                if (cell.IsOccupied())
                {
                    Vector2Int pos = cell.GetGridPosition();
                    queue.push(pos);
                    visited.insert(pos);
                }
            }

            const std::vector<Vector2Int> directions = {
                {1, 0}, {1, 1}, {0, 1}, {-1, 1},
                {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
            };

            int currentStep = 1;
            size_t nodesInCurrentLayer = queue.size();

            while (!queue.empty() && currentStep <= _costSteps)
            {
                const Vector2Int current = queue.front();
                queue.pop();
                nodesInCurrentLayer--;

                for (const auto& dir : directions)
                {
                    Vector2Int neighbour(current.x + dir.x, current.y + dir.y);

                    if (!InBounds(neighbour.x, neighbour.y) || visited.contains(neighbour))
                        continue;

                    double penalty = _inflationPenalty / (double)currentStep;

                    auto& neighbourCell = _grid[GetIndex(neighbour.x, neighbour.y)];
                    neighbourCell.SetCost(std::max(neighbourCell.GetCost(), penalty));

                    visited.insert(neighbour);
                    queue.push(neighbour);
                }

                if (nodesInCurrentLayer == 0)
                {
                    currentStep++;
                    nodesInCurrentLayer = queue.size();
                }
            }
        }


        static std::vector<Vector2Int> Bresenham(const Vector2Int start, const Vector2Int end)
        {
            std::vector<Vector2Int> path;
            int x0 = start.x;
            int y0 = start.y;
            const int x1 = end.x;
            const int y1 = end.y;

            const int dx = std::abs(x1 - x0);
            const int dy = std::abs(y1 - y0);

            const int sx = x0 < x1 ? 1 : -1;
            const int sy = y0 < y1 ? 1 : -1;

            int err = dx - dy;

            while (true)
            {
                path.emplace_back(x0, y0);

                if (x0 == x1 && y0 == y1)
                    break;

                const int e2 = 2 * err;

                if (e2 > -dy)
                {
                    err -= dy;
                    x0 += sx;
                }

                if (e2 < dx)
                {
                    err += dx;
                    y0 += sy;
                }
            }

            return path;
        }
    };
}
