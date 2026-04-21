#include "SlamController.hpp"
#include "LidarController.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace std;

namespace Manhattan::Core {
SlamController::SlamController(const App& app)
    : BaseController(app)
    , _grid(Vector2Int(200, 200), 0.05, 8, 20)
    , _poseMatcher(PoseMatcher(_grid, 5))
    , _lastStablePose(Pose::Identity())
    , _lastOdomPose(Pose::Identity())
    , _odomPoseDelta(Pose::Zero())
{
    app.GetController<LidarController>()->SetScanCallback(
        [this](const std::vector<Vector2>& points) {
            this->OnLidar(points);
        });

    _posePub = _node->create_publisher<geometry_msgs::msg::PoseStamped>("~/slam/pose", 5);
    _pathPub = _node->create_publisher<nav_msgs::msg::Path>("~/slam/path", 5);

    _gridPub = _node->create_publisher<nav_msgs::msg::OccupancyGrid>("~/slam/grid", 1);
    _gridMapPub = _node->create_publisher<grid_map_msgs::msg::GridMap>("~/slam/grid_map", 1);

    _path.header.frame_id = "map";

    _odometrySub = _node->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->OnOdometry(msg);
        });

    _publishTimer = _node->create_wall_timer(200ms, [this] { Publish(); });
    _costUpdateTimer = _node->create_wall_timer(1000ms, [this] { _grid.RecalculateCosts(); });

    RCLCPP_INFO(_node->get_logger(), "SlamController initialized");
}

void SlamController::OnOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    const auto current = Pose(Vector2(msg->pose.pose.position.x, msg->pose.pose.position.y),
        2.0 * std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) - M_PI * 0.5);

    {
        std::lock_guard guard(_odomLock);

        const auto delta = current - _lastOdomPose;
        std::cout << "odom  : " << delta.ToString() << std::endl;

        _odomPoseDelta = _odomPoseDelta + delta;

        _lastOdomPose = current;
    }
}

void SlamController::OnLidar(const std::vector<Vector2>& points)
{
    if (points.empty())
        return;

    std::lock_guard lock(_mapLock);

    // todo: support ResetGridIfNeeded() if robot leaves grid

    Pose odomDelta;

    {
        std::lock_guard guard(_odomLock);

        odomDelta = _odomPoseDelta;

        _odomPoseDelta = Pose::Zero();
    }

    const auto stableResult = _poseMatcher.Match(points, _lastStablePose);
    const auto odomResult = _poseMatcher.Match(points, _lastOdomPose + odomDelta);
    std::cout << "odomDelta  : " << odomDelta.ToString() << std::endl;

    const auto result = PoseResult::Combine(stableResult, odomResult);

    _lastStablePose = result.pose;

    std::cout << "Matcher confidence: " << result.confidence << std::endl;

    if (result.confidence < 0.5)
        return;

    MapScan(points);
}

void SlamController::MapScan(const std::vector<Vector2>& points)
{
    // Convert robot position to grid coordinates
    const auto startGridPos = _grid.WorldToGrid(_lastStablePose.position);

    const auto cosRot = std::cos(_lastStablePose.rotation);
    const auto sinRot = std::sin(_lastStablePose.rotation);

    // For each point in the scan
    for (const auto& p : points) {

        const auto point = Vector2(
            _lastStablePose.position.x + p.x * cosRot - p.y * sinRot,
            _lastStablePose.position.y + p.x * sinRot + p.y * cosRot);

        // Convert to grid coordinates
        const auto endGridPos = _grid.WorldToGrid(point);

        // Mark all cells along the line as free (using Bresenham algorithm)
        const auto bresenhamCells = OccupancyGrid::Bresenham(startGridPos, endGridPos);
        for (const auto& cell : bresenhamCells) {
            // Mark as free with distance-based cost
            const auto distance = std::sqrt((point.x - _lastStablePose.position.x) * (point.x - _lastStablePose.position.x) + (point.y - _lastStablePose.position.y) * (point.y - _lastStablePose.position.y));

            _grid.SetFree(cell, distance);
        }

        // Mark the endpoint as occupied
        _grid.SetOccupied(endGridPos);
    }
}

void SlamController::Publish()
{
    const std::unique_lock lock(_mapLock, std::try_to_lock);
    if (!lock.owns_lock())
        return;

    PublishPose(_lastStablePose);
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
            auto cell = _grid.GetCell({ x, y });
            auto cost = cell->GetCost();

            gridMsg.data[_grid.GetIndex(x, y)] = static_cast<int8_t>(cell->GetProbability() * 100.0);
        }
    }

    _gridPub->publish(gridMsg);
}

void SlamController::PublishGridMap()
{
    grid_map::GridMap map;

    const auto width = _grid.GetWidth();
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

GridCell* SlamController::GetCell(const Vector2& position)
{
    return _grid.GetCell(_grid.WorldToGrid(position));
}

std::vector<GridCell*> SlamController::GetNeighbors(const GridCell* cell)
{
    auto neighbours = std::vector<GridCell*>();

    for (auto direction : Vector2Int::Directions()) {
        auto neighbourCell = _grid.GetCell(cell->GetGridPosition() + direction);
        if (neighbourCell == nullptr)
            continue;

        neighbours.push_back(neighbourCell);
    }

    return neighbours;
}

bool SlamController::RayCast(const Vector2& worldPosition, const Vector2& direction, RayHit& rayHit, double maxDistance)
{
    const auto startCell = _grid.WorldToGrid(worldPosition);
    const auto endWorldPosition = worldPosition + direction.Normalized() * maxDistance;
    const auto endCell = _grid.WorldToGrid(endWorldPosition);

    rayHit = RayHit();

    for (const auto& pos : OccupancyGrid::Bresenham(startCell, endCell)) {
        const auto cell = _grid.GetCell(pos);
        if (cell == nullptr || !cell->IsOccupied())
            continue;

        rayHit.hit = _grid.GridToWorld(pos);

        auto normalInt = Vector2Int::Zero();

        for (auto dir : Vector2Int::Directions()) {
            const auto nCell = _grid.GetCell(pos + dir);

            if (nCell != nullptr && nCell->IsOccupied())
                continue;

            normalInt = normalInt + dir;
        }

        if (normalInt != Vector2Int::Zero()) {
            rayHit.normal = Vector2(normalInt).Normalized();

            if (Vector2::Dot(rayHit.normal, direction) > 0.0)
                rayHit.normal = -rayHit.normal;

            return true;
        }

        const auto toStart = Vector2(startCell - pos);

        rayHit.normal = toStart.SqrMagnitude() > 0.0 ? toStart.Normalized() : -direction.Normalized();

        return true;
    }

    return false;
}
} // namespace Manhattan::Core
