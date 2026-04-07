#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

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

        OccupancyGrid _grid;
        PoseMatcher _poseMatcher;
        Pose _lastPose;

        std::mutex _updateMutex;

        void Update(const std::vector<Point> &points);

        std::vector<Point> TransformPointsLocalToWorld(const std::vector<Point>& localPoints, const Pose& pose) const;

        void MapScan(const std::vector<Point>& worldPoints, const Point& robotPosition);

        void PublishPose(const Pose& pose);

        void PublishGrid();
    };
}
