#include "SlamController.hpp"
#include "LidarController.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace std;

namespace Manhattan::Core
{
    SlamController::SlamController(const App& app)
        : BaseController(app),
        _grid(Vector2Int(200, 200), 0.05, 8, 20),
        _poseMatcher(PoseMatcher(_grid, 5)),
        _lastPose(Pose(Vector2(0, 0), 0))
    {
        app.GetController<LidarController>()->SetScanCallback(
        [this](const std::vector<Vector2>& points) { this->Update(points); }
        );

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

        _timer = _node->create_wall_timer(
            100ms,
        [this] { Publish(); }
        );

        RCLCPP_INFO(_node->get_logger(), "SlamController initialized");
    }

    void SlamController::Update(const std::vector<Vector2> &points)
    {
        const std::unique_lock lock(_updateMutex, std::try_to_lock);

        if (!lock.owns_lock()) {
            RCLCPP_DEBUG(_node->get_logger(), "SlamController: Update skipped (already running)");
            return;
        }
        if (points.empty()) return;

        // ResetGridIfNeeded();

        _lastPose = _poseMatcher.Match(points, _lastPose.position, _lastPose.rotation);

        auto worldPoints = TransformPointsLocalToWorld(points, _lastPose);

        MapScan(worldPoints, _lastPose.position);

        _grid.RecalculateCosts();
    }

    std::vector<Vector2> SlamController::TransformPointsLocalToWorld(
        const std::vector<Vector2>& localPoints,
        const Pose& pose) const
    {
        std::vector<Vector2> worldPoints;
        worldPoints.reserve(localPoints.size());

        auto cosRot = std::cos(pose.rotation);
        auto sinRot = std::sin(pose.rotation);

        for (const auto& localPoint : localPoints) {
            Vector2 worldPoint{
                pose.position.x + localPoint.x * cosRot - localPoint.y * sinRot,
                pose.position.y + localPoint.x * sinRot + localPoint.y * cosRot
            };

            worldPoints.push_back(worldPoint);
        }

        return worldPoints;
    }

    void SlamController::MapScan(
        const std::vector<Vector2>& worldPoints,
        const Vector2& robotPosition)
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

    void SlamController::Publish() {
        const std::unique_lock lock(_updateMutex, std::try_to_lock);

        if (!lock.owns_lock()) return;

        PublishPose(_lastPose);
        PublishGrid();
        PublishGridMap();
    }

    void SlamController::PublishPose(const Pose& pose)
    {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = _node->now();
        pose_msg.header.frame_id = "map";

        pose_msg.pose.position.x = pose.position.x;
        pose_msg.pose.position.y = pose.position.y;
        pose_msg.pose.position.z = 0.0;

        // Convert rotation angle to quaternion
        double halfRotation = (pose.rotation + M_PI * 0.5) * 0.5;

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

    GridCell* SlamController::GetCell(const Vector2 &position) {
        return _grid.GetCell(_grid.WorldToGrid(position));
    }


    std::vector<GridCell*> SlamController::GetNeighbors(const GridCell* cell) {
        auto neighbours = std::vector<GridCell*>();

        for (auto direction : Vector2Int::Directions()) {
            auto neighbourCell = _grid.GetCell(cell->GetGridPosition() + direction);
            if (neighbourCell == nullptr) continue;

            neighbours.push_back(neighbourCell);
        }

        return neighbours;
    }

    bool SlamController::RayCast(const Vector2 &worldPosition, const Vector2 &direction, RayHit &rayHit, double maxDistance) {
        const auto startCell = _grid.WorldToGrid(worldPosition);
        const auto endWorldPosition = worldPosition + direction.Normalized() * maxDistance;
        const auto endCell = _grid.WorldToGrid(endWorldPosition);

        rayHit = RayHit();

        for (const auto &pos : OccupancyGrid::Bresenham(startCell, endCell)) {
            const auto cell = _grid.GetCell(pos);
            if (cell == nullptr || !cell->IsOccupied()) continue;

            rayHit.hit = _grid.GridToWorld(pos);

            auto normalInt = Vector2Int::Zero();

            for (auto dir : Vector2Int::Directions()) {
                const auto nCell = _grid.GetCell(pos + dir);

                if (nCell != nullptr && nCell->IsOccupied()) continue;

                normalInt = normalInt + dir;
            }

            if (normalInt != Vector2Int::Zero()) {
                rayHit.normal = Vector2(normalInt).Normalized();

                if (Vector2::Dot(rayHit.normal, direction) > 0.0)
                    rayHit.normal = -rayHit.normal;

                return true;
            }

            const auto toStart = Vector2(startCell - pos);

            rayHit.normal = toStart.SqrMagnitude() > 0.0
                ? toStart.Normalized()
                : -direction.Normalized();

            return true;
        }

        return false;

    }
}
