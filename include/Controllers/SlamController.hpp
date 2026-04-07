#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "App.h"
#include "PoseMatcher.hpp"
#include "OccupancyGrid.hpp"
#include "Point.hpp"


namespace Manhattan::Core {
    class SlamController final : public BaseController {
    public:
        explicit SlamController(const App& app);

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometrySub;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _posePub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pathPub;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _gridPub;
        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _gridMapPub;

        OccupancyGrid _grid;
        PoseMatcher _poseMatcher;
        Pose _lastPose;

        nav_msgs::msg::Path _path;

        std::mutex _updateMutex;

        void Update(const std::vector<Point> &points);

        std::vector<Point> TransformPointsLocalToWorld(const std::vector<Point>& localPoints, const Pose& pose) const;

        void MapScan(const std::vector<Point>& worldPoints, const Point& robotPosition);

        void PublishPose(const Pose& pose);

        void PublishGrid();
        void PublishGridMap();

    };
}
