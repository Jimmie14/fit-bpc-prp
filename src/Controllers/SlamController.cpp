#include "SlamController.hpp"
#include "LidarController.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace std;

namespace Manhattan::Core
{
    SlamController::SlamController(const App& app)
        : BaseController(app),
        _grid(Vector2Int(200, 200), 0.05, 5, 20),
        _poseMatcher(PoseMatcher(_grid, 5))
    {
        app.GetController<LidarController>()->SetScanCallback(
        [this](const std::vector<Point>& points) { this->Update(points); }
        );

        _lastPose = { Point(0, 0), 0 };

        _posePub = _node->create_publisher<geometry_msgs::msg::PoseStamped>("~/slam/pose", 10);
        _pathPub = _node->create_publisher<nav_msgs::msg::Path>("~/slam/path", 10);

        _gridPub = _node->create_publisher<nav_msgs::msg::OccupancyGrid>("~/slam/grid", 10);
        _gridMapPub = _node->create_publisher<grid_map_msgs::msg::GridMap>("~/slam/grid_map", 10);

        _path.header.frame_id = "map";

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
        PublishGridMap();
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
        double halfRotation = (pose.Rotation + M_PI * 0.5) * 0.5;

        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = std::sin(halfRotation);
        pose_msg.pose.orientation.w = std::cos(halfRotation);


        _path.poses.push_back(pose_msg);
        if (_path.poses.size() > 1000)
            _path.poses.erase(_path.poses.begin());

        _pathPub->publish(_path);


        _posePub->publish(pose_msg);
    }


    void SlamController::PublishGrid()
    {
        nav_msgs::msg::OccupancyGrid gridMsg;
        gridMsg.header.stamp = _node->now();
        gridMsg.header.frame_id = "map";

        gridMsg.info.origin.position.x = _grid.GetWidth() * _grid.GetCellSize() * -0.5;
        gridMsg.info.origin.position.y = _grid.GetWidth() * _grid.GetCellSize() * -0.5;
        gridMsg.info.origin.position.z = 0.0;

        gridMsg.info.origin.orientation.x = 0.0;
        gridMsg.info.origin.orientation.y = 0.0;
        gridMsg.info.origin.orientation.z = 0.0;
        gridMsg.info.origin.orientation.w = 1.0;

        gridMsg.info.width = _grid.GetWidth();
        gridMsg.info.height = _grid.GetHeight();
        gridMsg.info.resolution = static_cast<float>(_grid.GetCellSize());

        const auto size = gridMsg.info.width * gridMsg.info.height;
        gridMsg.data.resize(size);

        for (auto x = 0; x < gridMsg.info.width; x++) {
            for (auto y = 0; y < gridMsg.info.height; y++) {
                auto cell = _grid.GetCell({x, y});
                auto cost = cell->GetCost();

                gridMsg.data[_grid.GetIndex(x, y)] = static_cast<int8_t>(cell->GetProbability() * 100.0);
            }
        }

        _gridPub->publish(gridMsg);
    }

    void SlamController::PublishGridMap()
    {
        grid_map::GridMap map;

        const auto width  = _grid.GetWidth();
        const auto height = _grid.GetHeight();
        const auto resolution = _grid.GetCellSize();

        const auto size_x = width * resolution;
        const auto size_y = height * resolution;

        // -----------------------------
        // Initialize grid map
        // -----------------------------
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(size_x, size_y), resolution, grid_map::Position(0.0, 0.0));

        map.add("probability");
        map.add("cost");

        // -----------------------------
        // Fill data
        // -----------------------------
        for (auto x = 0; x < width; x++) {
            for (auto y = 0; y < height; y++) {

                const auto cell = _grid.GetCell({ x, y });

                const auto probability = cell->GetProbability();
                const auto cost = cell->GetCost();

                const grid_map::Index index(width - 1 - x, height - 1 - y);

                map.at("probability", index) = static_cast<float>(probability);
                map.at("cost", index) = static_cast<float>(cost);
            }
        }

        // -----------------------------
        // Convert + publish
        // -----------------------------
        const auto msg = grid_map::GridMapRosConverter::toMessage(map);

        _gridMapPub->publish(*msg);
    }

}
