#include "Components/MapThinningUnit.hpp"

#include "App.hpp"
#include "Nav/Grid.hpp"
#include "Viz/Grid.hpp"

namespace Manhattan::core {

Grid<bool> ApplyMedianFilter(const Grid<bool>& grid);

Grid<bool> ThickenWalls(const Grid<bool>& grid);

Grid<bool> ZhangSuenThinning(const Grid<bool>& grid);

int CountNeighbors(const Grid<bool>& grid, const int x, const int y);

int CountTransitions(const Grid<bool>& grid, const int x, const int y);

MapThinningUnit::MapThinningUnit(const App& app)
    : RosUnit(app, "map_thinning")
{
}

void MapThinningUnit::OnEnable()
{
    _mapSubscription = create_subscription<nav_msgs::msg::OccupancyGrid>("slam/grid", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->OnMap(msg);
    });

    _mapPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("slam/grid_thinned", 1);
}

void MapThinningUnit::OnDisable()
{
    _mapSubscription.reset();
    _mapPublisher.reset();
}

void MapThinningUnit::OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg) const
{
    auto grid = viz::nav::ToOccupancyGrid(*msg, 20);

    grid = ThickenWalls(grid);
    grid = ApplyMedianFilter(grid);
    grid = ZhangSuenThinning(grid);

    _app.events->Publish(ThinnedMapEvent { grid });
    _mapPublisher->publish(viz::nav::ToOccupancyGridMessage(grid, "map"));
}

Grid<bool> ApplyMedianFilter(const Grid<bool>& grid)
{
    auto result = Grid<bool>(grid.width(), grid.height(), grid.resolution());

    for (int x = 1; x < grid.width() - 1; x++) {
        for (int y = 1; y < grid.height() - 1; y++) {
            int trueCount = 0;

            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (grid.get(x + i, y + j)) {
                        trueCount++;
                    }
                }
            }

            result.set(x, y, trueCount >= 5);
        }
    }

    return result;
}

Grid<bool> ThickenWalls(const Grid<bool>& grid)
{
    auto result = grid;

    for (auto i = 0; i < grid.size(); i++) {
        if (grid[i]) continue;

        const auto [x, y] = grid.indexToCoord(i);

        result.setChecked(x + 1, y, false);
        result.setChecked(x - 1, y, false);
        result.setChecked(x, y + 1, false);
        result.setChecked(x, y - 1, false);
    }

    return result;
}

Grid<bool> ZhangSuenThinning(const Grid<bool>& grid)
{
    Grid<bool> skeleton = grid;

    bool changed;
    do {
        changed = false;
        std::vector<std::pair<int, int>> toRemove;

        for (int step = 0; step < 2; step++) {
            toRemove.clear();
            for (int x = 1; x < grid.width() - 1; x++) {
                for (int y = 1; y < grid.height() - 1; y++) {
                    if (!skeleton.get(x, y)) continue;

                    const auto B = CountNeighbors(skeleton, x, y);
                    const auto A = CountTransitions(skeleton, x, y);

                    const auto p2 = skeleton.get(x, y + 1);
                    const auto p4 = skeleton.get(x + 1, y);
                    const auto p6 = skeleton.get(x, y - 1);
                    const auto p8 = skeleton.get(x - 1, y);

                    bool condition;
                    if (step == 0)
                        condition = !(p2 && p4 && p6) && !(p4 && p6 && p8);
                    else
                        condition = !(p2 && p4 && p8) && !(p2 && p6 && p8);

                    if (B >= 2 && B <= 6 && A == 1 && condition) {
                        toRemove.push_back({ x, y });
                        changed = true;
                    }
                }
            }

            for (auto& [x, y] : toRemove)
                skeleton.set(x, y, false);
        }
    } while (changed);

    return skeleton;
}

int CountNeighbors(const Grid<bool>& grid, const int x, const int y)
{
    int count = 0;

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0) continue;

            if (grid(x + j, y + i))
                count++;
        }
    }

    return count;
}

int CountTransitions(const Grid<bool>& grid, const int x, const int y)
{
    const bool p[8] = {
        grid(x, y + 1), grid(x + 1, y + 1), grid(x + 1, y), grid(x + 1, y - 1),
        grid(x, y - 1), grid(x - 1, y - 1), grid(x - 1, y), grid(x - 1, y + 1)
    };

    int transitions = 0;

    for (int i = 0; i < 8; i++) {
        if (p[i] || !p[(i + 1) % 8]) continue;

        transitions++;
    }

    return transitions;
}

} // namespace Manhattan::Core
