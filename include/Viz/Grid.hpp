#pragma once

#include "Nav/Grid.hpp"
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <rclcpp/rclcpp.hpp>

namespace Manhattan::Nav::Viz {
inline Grid<double> ToProbabilityGrid(const nav_msgs::msg::OccupancyGrid& grid)
{
    auto result = Grid<double>(grid.info.width, grid.info.height, grid.info.resolution);

    for (auto i = 0; i < result.size; i++) {
        const auto [x, y] = result.indexToCoord(i);

        result.set(i, static_cast<double>(grid.data[y * grid.info.width + x]) / 126.0);
    }

    return result;
}

inline std::shared_ptr<Grid<bool>> ToOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid, const signed char threshold)
{
    auto result = std::make_shared<Grid<bool>>(grid.info.width, grid.info.height, grid.info.resolution);

    for (auto i = 0; i < result->size; i++) {
        const auto [x, y] = result->indexToCoord(i);

        const auto unknown = grid.data[y * grid.info.width + x] < 0;
        const auto free = grid.data[y * grid.info.width + x] <= threshold;

        result->set(x, y, !unknown && free);
    }
    return result;
}

inline nav_msgs::msg::OccupancyGrid ToOccupancyGridMessage(const Grid<bool>& grid, const string& frameId)
{
    nav_msgs::msg::OccupancyGrid result;
    result.header.stamp = rclcpp::Clock().now();
    result.header.frame_id = frameId;

    result.info.origin.position.x = grid.width * grid.resolution * -0.5;
    result.info.origin.position.y = grid.height * grid.resolution * -0.5;
    result.info.origin.position.z = 0.0;

    result.info.origin.orientation.x = 0.0;
    result.info.origin.orientation.y = 0.0;
    result.info.origin.orientation.z = 0.0;
    result.info.origin.orientation.w = 1.0;

    result.info.width = grid.width;
    result.info.height = grid.height;
    result.info.resolution = grid.resolution;

    result.data.resize(grid.size);

    for (auto i = 0; i < grid.size; i++) {
        const auto [x, y] = grid.indexToCoord(i);

        result.data[y * grid.width + x] = grid(x, y) ? 100 : 0;
    }

    return result;
}

}
