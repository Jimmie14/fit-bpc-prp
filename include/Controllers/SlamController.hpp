#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "App.h"
#include "PoseMatcher.hpp"
#include "OccupancyGrid.hpp"
#include "Vector2.hpp"


namespace Manhattan::Core {
    class SlamController final : public BaseController {
    public:
        explicit SlamController(const App& app);
        GridCell* GetCell(const Vector2 &position);
        std::vector<GridCell*> GetNeighbors(const GridCell* cell);
        bool RayCast(const Vector2 &worldPosition, const Vector2 &direction, RayHit &rayHit, double maxDistance = 100);

        [[nodiscard]] Pose CurrentPose() const {
            return _lastStablePose;
        }

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometrySub;

        rclcpp::TimerBase::SharedPtr _timer;

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

        std::mutex _lidarLock;
        std::mutex _odomLock;

        void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr &msg);

        void OnLidar(const std::vector<Vector2> &points);

        [[nodiscard]] std::vector<Vector2> TransformPointsLocalToWorld(const std::vector<Vector2>& localPoints, const Pose& pose) const;

        void MapScan(const std::vector<Vector2>& worldPoints, const Vector2& robotPosition);

        void Publish();
        void PublishPose(const Pose& pose);

        void PublishGrid();
        void PublishGridMap();

    };
}
