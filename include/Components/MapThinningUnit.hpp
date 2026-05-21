#pragma once

#include "Common/RosUnit.hpp"
#include "Nav/Grid.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace Manhattan::nav;

namespace Manhattan::core {

struct ThinnedMapEvent {
    Grid<bool> grid;
};

class MapThinningUnit : public RosUnit {
public:
    explicit MapThinningUnit(const App& app);

protected:
    void OnEnable() override;

    void OnDisable() override;

private:
    Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _mapSubscription;

    Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _mapPublisher;

    void OnMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg) const;
};
}