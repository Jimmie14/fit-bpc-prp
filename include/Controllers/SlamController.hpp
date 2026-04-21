#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "App.h"
#include "OccupancyGrid.hpp"
#include "PoseMatcher.hpp"
#include "Vector2.hpp"

namespace Manhattan::Core {
class SlamController final : public BaseController {
public:
    explicit SlamController(const App& app);
    GridCell* GetCell(const Vector2& position);
    std::vector<GridCell*> GetNeighbors(const GridCell* cell);
    bool RayCast(const Vector2& worldPosition, const Vector2& direction, RayHit& rayHit, double maxDistance = 100);

    [[nodiscard]] Pose CurrentPose() const
    {
        return _lastStablePose;
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometrySub;

    rclcpp::TimerBase::SharedPtr _publishTimer;
    rclcpp::TimerBase::SharedPtr _costUpdateTimer;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _posePub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pathPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _gridPub;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _gridMapPub;

    OccupancyGrid _grid;
    PoseMatcher _poseMatcher;

    Pose _lastOdomPose;
    Pose _odomPoseDelta;

    Pose _lastStablePose;

    nav_msgs::msg::Path _path;

    std::mutex _mapLock;
    std::mutex _odomLock;

    void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg);

    void OnLidar(const std::vector<Vector2>& points);

    void MapScan(const std::vector<Vector2>& points);

    void Publish();
    void PublishPose(const Pose& pose);

    void PublishGrid();
    void PublishGridMap();
};
} // namespace Manhattan::Core
