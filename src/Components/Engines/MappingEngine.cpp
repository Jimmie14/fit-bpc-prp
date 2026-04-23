#include "MappingEngine.hpp"
#include "LidarDriver.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <random>

using namespace std;

constexpr auto minConfidence = 0.6;

namespace Manhattan::Core {
MappingEngine::MappingEngine(App& app)
    : RosEngine(app, "mapping")
    , _grid(Vector2Int(200, 200), 0.05, 8, 20)
    , _poseMatcher(PoseMatcher(_grid, 5))
    , _lastOdomPose(Pose::Identity())
    , _hypotheses({ PoseMatchResult(Pose::Identity(), minConfidence) })
{
    _lostTime = now();

    app.Events->Subscribe<LidarScan>([this](const LidarScan& scan) {
        this->OnLidar(scan.points);
    });

    _posePublisher = create_publisher<geometry_msgs::msg::PoseStamped>("slam/pose", 1);
    _pathPublisher = create_publisher<nav_msgs::msg::Path>("slam/path", 1);

    _hypoPublisher = create_publisher<geometry_msgs::msg::PoseArray>("slam/hypo", 1);

    _gridPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("slam/grid", 1);
    _gridMapPublisher = create_publisher<grid_map_msgs::msg::GridMap>("slam/grid_map", 1);

    _path.header.frame_id = "map";

    _odometrySub = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->OnOdometry(msg);
        });

    _publishTimer = create_wall_timer(200ms, [this] {
        Publish();
    });
    _costUpdateTimer = create_wall_timer(1000ms, [this] {
        std::lock_guard guard(_mapLock);

        this->_grid.RecalculateCosts();
    });

    RCLCPP_INFO(get_logger(), "SlamController initialized");
}

void MappingEngine::OnOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    const auto current = Pose(Vector2(msg->pose.pose.position.x, msg->pose.pose.position.y),
        2.0 * std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) - M_PI * 0.5);

    {
        std::lock_guard guard(_odomLock);

        const auto delta = current - _lastOdomPose;
        // std::cout << "odom  : " << delta.ToString() << std::endl;

        _odomPoseDelta = _odomPoseDelta + delta;

        _lastOdomPose = current;
    }
}

void MappingEngine::OnLidar(const vector<Vector2>& points)
{
    if (points.empty())
        return;

    Pose odomDelta;
    {
        std::lock_guard guard(_odomLock);

        odomDelta = _odomPoseDelta;
        _odomPoseDelta = Pose::Zero();
    }

    std::lock_guard lock(_mapLock);

    _lastScan = points;

    switch (_state) {
    case MappingEngineState::Initializing:
        break;
    case MappingEngineState::Stable:
    case MappingEngineState::Degraded:
    case MappingEngineState::Lost:
        UpdateHypotheses(odomDelta);
        break;
    }

    MapScan(points);
}

void MappingEngine::UpdateHypotheses(const Pose& odomDelta)
{
    for (auto& hypothesis : _hypotheses) {
        auto stableResult = _poseMatcher.Match(_lastScan, hypothesis.pose);
        auto odomResult = _poseMatcher.Match(_lastScan, hypothesis.pose + odomDelta);

        hypothesis = PoseMatchResult::Best(stableResult, odomResult);
    }

    erase_if(_hypotheses, [](const PoseMatchResult& result) {
        return result.confidence < minConfidence;
    });

    if (_state == MappingEngineState::Degraded || _hypotheses.empty())
        CreateHypothesis();

    UpdateState();
}

void MappingEngine::UpdateState()
{
    constexpr auto lostTimeout = 10000ms;

    const auto timeNow = now();

    if (_hypotheses.empty()) {
        if (_state != MappingEngineState::Lost) {
            _state = MappingEngineState::Lost;
            _lostTime = timeNow;
            return;
        }

        if (timeNow - _lostTime > lostTimeout)
            Reset();

        return;
    }

    auto best = max_element(_hypotheses.begin(), _hypotheses.end(), [](const auto& a, const auto& b) {
        return a.confidence < b.confidence;
    });

    std::cout << "best conf : " << best->confidence << std::endl;
    for (const auto& hypothesis : _hypotheses) {
        std::cout << "conf       : " << hypothesis.confidence << std::endl;
    }

    if (!_hypotheses.empty() && best->confidence >= 0.8) {
        _state = MappingEngineState::Stable;
        _stablePose = best->pose;
        return;
    }

    if (_hypotheses.size() < 2) {
        _state = MappingEngineState::Degraded;
        _stablePose = best->pose;
        return;
    }

    if (_state != MappingEngineState::Lost) {
        _state = MappingEngineState::Lost;
        _lostTime = timeNow;
        return;
    }

    if (timeNow - _lostTime > lostTimeout)
        Reset();
}

void MappingEngine::CreateHypothesis()
{
    if (_hypotheses.empty()) {
        RCLCPP_INFO(get_logger(), "MCL failed no hypothesis found.");
        _hypotheses = { PoseMatchResult(Pose::Identity(), minConfidence) };
        return;
    }

    const auto best = ranges::max_element(_hypotheses, [](const auto& a, const auto& b) {
        return a.confidence < b.confidence;
    });

    if (best == _hypotheses.end())
        return;

    static thread_local auto prng = std::mt19937(std::random_device {}());
    constexpr auto attempts = 20;

    auto offsetXY = std::uniform_real_distribution(-0.75, 0.75);
    auto offsetTheta = std::uniform_real_distribution(-0.35, 0.35);

    for (auto i = 0; i < attempts; i++) {
        const auto candidatePose = Pose(
            Vector2(best->pose.position.x + offsetXY(prng), best->pose.position.y + offsetXY(prng)),
            best->pose.rotation + offsetTheta(prng));

        const auto hypothesis = _poseMatcher.Match(_lastScan, candidatePose);
        if (hypothesis.confidence < minConfidence)
            continue;

        _hypotheses.push_back(hypothesis);
    }
}

void MappingEngine::Reset()
{
    _state = MappingEngineState::Initializing;
    _hypotheses = { PoseMatchResult(Pose::Identity(), minConfidence) };
    _grid.Reset();

    _lastStoredPose = Pose::Identity();
    _stablePose = Pose::Identity();
}

void MappingEngine::MapScan(const std::vector<Vector2>& points)
{
    if (_state != MappingEngineState::Stable && _state != MappingEngineState::Initializing)
        return;

    if (_state == MappingEngineState::Initializing) {
        _state = MappingEngineState::Lost;
    }

    // Convert robot position to grid coordinates
    const auto startGridPos = _grid.WorldToGrid(_stablePose.position);

    const auto cosRot = std::cos(_stablePose.rotation);
    const auto sinRot = std::sin(_stablePose.rotation);

    // For each point in the scan
    for (const auto& p : points) {

        const auto point = Vector2(
            _stablePose.position.x + p.x * cosRot - p.y * sinRot,
            _stablePose.position.y + p.x * sinRot + p.y * cosRot);

        // Convert to grid coordinates
        const auto endGridPos = _grid.WorldToGrid(point);

        // Mark all cells along the line as free (using Bresenham algorithm)
        const auto bresenhamCells = OccupancyGrid::Bresenham(startGridPos, endGridPos);
        for (const auto& cell : bresenhamCells) {
            // Mark as free with distance-based cost
            const auto distance = std::sqrt((point.x - _stablePose.position.x) * (point.x - _stablePose.position.x) + (point.y - _stablePose.position.y) * (point.y - _stablePose.position.y));

            _grid.SetFree(cell, distance);
        }

        // Mark the endpoint as occupied
        _grid.SetOccupied(endGridPos);
    }
}

void MappingEngine::Publish()
{
    const std::unique_lock lock(_mapLock, std::try_to_lock);
    if (!lock.owns_lock())
        return;

    PublishPose();
    PublishGrid();
    PublishGridMap();
}

void MappingEngine::PublishPose()
{
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.stamp = now();
    msg.header.frame_id = "map";

    for (const auto hypothesis : _hypotheses) {
        msg.poses.push_back(hypothesis.pose.ToRosPoseMessage());
    }

    _hypoPublisher->publish(msg);

    auto poseMsg = geometry_msgs::msg::PoseStamped();

    poseMsg.header.stamp = now();
    poseMsg.header.frame_id = "map";

    poseMsg.pose = _stablePose.ToRosPoseMessage();

    _posePublisher->publish(poseMsg);

    _path.poses.push_back(poseMsg);
    if (_path.poses.size() > 100)
        _path.poses.erase(_path.poses.begin());

    _pathPublisher->publish(_path);
}

void MappingEngine::PublishGrid()
{
    nav_msgs::msg::OccupancyGrid gridMsg;
    gridMsg.header.stamp = now();
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

    _gridPublisher->publish(gridMsg);
}

void MappingEngine::PublishGridMap()
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

    _gridMapPublisher->publish(*msg);
}

GridCell* MappingEngine::GetCell(const Vector2& position)
{
    return _grid.GetCell(_grid.WorldToGrid(position));
}

GridCell* MappingEngine::GetCell(Vector2Int gridPosition)
{
    return _grid.GetCell(gridPosition);
}

Vector2 MappingEngine::GridToWorld(const Vector2Int& pos) const
{
    return _grid.GridToWorld(pos);
}

std::vector<GridCell*> MappingEngine::GetNeighbors(const GridCell* cell)
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

bool MappingEngine::RayCast(const Vector2& worldPosition, const Vector2& direction, RayHit& rayHit, double maxDistance)
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
