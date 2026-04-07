#include "SlamController.hpp"
#include "LidarController.hpp"

using namespace std;

namespace Manhattan::Core
{
    SlamController::SlamController(const App& app)
        : BaseController(app),
        _grid(Vector2Int(100, 100), 0.2, 5),
        _poseMatcher(PoseMatcher(_grid, 5))
    {
        app.GetController<LidarController>()->SetScanCallback(
        [this](const std::vector<Point>& points) { this->Update(points); }
        );

        _lastPose = { Point(0, 0), 0 };

        _posePub = _node->create_publisher<geometry_msgs::msg::PoseStamped>("~/slam/pose", 10);
        _pathPub = _node->create_publisher<nav_msgs::msg::Path>("~/slam/path", 10);
        _gridPub = _node->create_publisher<nav_msgs::msg::OccupancyGrid>("~/slam/grid", 10);

        // _odometrySub = _node->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odometry/filtered",
        //     10,
        //     [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        //         // Handle odometry updates if needed
        //     }
        // );

        RCLCPP_INFO(_node->get_logger(), "SlamController initialized");
    }

    void SlamController::Update(const std::vector<Point> &points)
    {
        const std::unique_lock lock(_updateMutex, std::try_to_lock);

        if (!lock.owns_lock()) {
            RCLCPP_DEBUG(_node->get_logger(), "SlamController: Update skipped (already running)");
            return;
        }
        if (points.empty()) return;

        // ResetGridIfNeeded();

        _lastPose = _poseMatcher.Match(points, _lastPose.Position, _lastPose.Rotation);

        auto worldPoints = TransformPointsLocalToWorld(points, _lastPose);

        MapScan(worldPoints, _lastPose.Position);

        _grid.RecalculateCosts();

        PublishPose(_lastPose);
        PublishGrid();

        RCLCPP_INFO(_node->get_logger(),
            "SLAM update: pose=(%.2f, %.2f), rotation=%.2f",
            _lastPose.Position.x, _lastPose.Position.y, _lastPose.Rotation);
    }

    std::vector<Point> SlamController::TransformPointsLocalToWorld(
        const std::vector<Point>& localPoints,
        const Pose& pose) const
    {
        std::vector<Point> worldPoints;
        worldPoints.reserve(localPoints.size());

        auto cosRot = std::cos(pose.Rotation);
        auto sinRot = std::sin(pose.Rotation);

        for (const auto& localPoint : localPoints) {
            Point worldPoint{
                pose.Position.x + localPoint.x * cosRot - localPoint.y * sinRot,
                pose.Position.y + localPoint.x * sinRot + localPoint.y * cosRot
            };

            worldPoints.push_back(worldPoint);
        }

        return worldPoints;
    }

    void SlamController::MapScan(
        const std::vector<Point>& worldPoints,
        const Point& robotPosition)
    {
        // Convert robot position to grid coordinates
        auto startGridPos = _grid.WorldToGrid(robotPosition);

        // For each point in the scan
        for (const auto& point : worldPoints) {
            // Convert to grid coordinates
            auto endGridPos = _grid.WorldToGrid(point);

            // Mark all cells along the line as free (using Bresenham algorithm)
            auto bresenhamCells = OccupancyGrid::Bresenham(startGridPos, endGridPos);
            for (const auto& cell : bresenhamCells) {
                // Mark as free with distance-based cost
                auto distance = std::sqrt(
                    (point.x - robotPosition.x) * (point.x - robotPosition.x) +
                    (point.y - robotPosition.y) * (point.y - robotPosition.y)
                );

                _grid.SetFree(cell, distance);
            }

            // Mark the endpoint as occupied
            _grid.SetOccupied(endGridPos);
        }
    }

    void SlamController::PublishPose(const Pose& pose)
    {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = _node->now();
        pose_msg.header.frame_id = "map";

        pose_msg.pose.position.x = pose.Position.x;
        pose_msg.pose.position.y = pose.Position.y;
        pose_msg.pose.position.z = 0.0;

        // Convert rotation angle to quaternion
        double halfRotation = pose.Rotation * 0.5;
        pose_msg.pose.orientation.w = std::cos(halfRotation);
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = std::sin(halfRotation);

        _posePub->publish(pose_msg);
    }

    void SlamController::PublishGrid()
    {
        nav_msgs::msg::OccupancyGrid gridMsg;
        gridMsg.header.stamp = _node->now();
        gridMsg.header.frame_id = "map";

        gridMsg.info.width = _grid.GetWidth();
        gridMsg.info.height = _grid.GetHeight();
        gridMsg.info.resolution = _grid.GetCellSize();

        const auto size = gridMsg.info.width * gridMsg.info.height;
        gridMsg.data.resize(size);

        for (auto x = 0; x < gridMsg.info.width; x++) {
            for (auto y = 0; y < gridMsg.info.height; y++) {
                auto probability = _grid.GetProbability(x, y);
                gridMsg.data[_grid.GetIndex(x, y)] = static_cast<int8_t>(probability * 100.0);
            }
        }

        _gridPub->publish(gridMsg);
    }
}


