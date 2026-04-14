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
            return _lastPose;
        }

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

        void Update(const std::vector<Vector2> &points);

        [[nodiscard]] std::vector<Vector2> TransformPointsLocalToWorld(const std::vector<Vector2>& localPoints, const Pose& pose) const;

        void MapScan(const std::vector<Vector2>& worldPoints, const Vector2& robotPosition);

        void PublishPose(const Pose& pose);

        void PublishGrid();
        void PublishGridMap();

    };
}
