#include "MappingEngine.hpp"
#include "LidarDriver.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace std;

namespace Manhattan::Core {
MappingEngine::MappingEngine(App& app)
    : RosEngine(app)
    , _grid(Vector2Int(200, 200), 0.05, 8, 20)
    , _poseMatcher(PoseMatcher(_grid, 5))
    , _lastOdomPose(Pose::Identity())
    , _hypotheses({ PoseMatchResult(Pose::Identity(), 0.5) })
{
    _lostTime = _node->now();

    app.Events.Subscribe<LidarScan>([this](const LidarScan& scan) {
        this->OnLidar(scan.points);
    });

    _scanPublisher = _node->create_publisher<sensor_msgs::msg::PointCloud2>("~/slam/scan", rclcpp::QoS(1).best_effort().durability_volatile());
    _posePublisher = _node->create_publisher<geometry_msgs::msg::PoseArray>("~/slam/pose", rclcpp::QoS(1));
    _pathPublisher = _node->create_publisher<nav_msgs::msg::Path>("~/slam/path", rclcpp::QoS(1));

    _gridPublisher = _node->create_publisher<nav_msgs::msg::OccupancyGrid>("~/slam/grid", rclcpp::QoS(1));
    _gridMapPublisher = _node->create_publisher<grid_map_msgs::msg::GridMap>("~/slam/grid_map", rclcpp::QoS(1).best_effort().durability_volatile());

    _path.header.frame_id = "map";

    _odometrySub = _node->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->OnOdometry(msg);
        });

    _publishTimer = _node->create_wall_timer(200ms, [this] {
        Publish();
    });
    _costUpdateTimer = _node->create_wall_timer(1000ms, [this] {
        std::lock_guard guard(_mapLock);

        this->_grid.RecalculateCosts();
    });

    RCLCPP_INFO(_node->get_logger(), "SlamController initialized");
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

        hypothesis = PoseMatchResult::Combine(stableResult, odomResult);
    }

    erase_if(_hypotheses, [](const PoseMatchResult& result) {
        return result.confidence < 0.5;
    });

    if (_hypotheses.empty())
        TryRelocalize();

    UpdateState();

    if (_state != MappingEngineState::Stable)
        return;

    const auto delta = _stablePose - _lastStoredPose;

    if (delta.position.Magnitude() > 0.5) {
        StoreScanContext();
        _lastStoredPose = _stablePose;
    }
}

void MappingEngine::UpdateState()
{
    constexpr auto lostTimeout = 10000ms;

    const auto now = _node->now();

    if (_hypotheses.empty()) {
        if (_state != MappingEngineState::Lost) {
            _state = MappingEngineState::Lost;
            _lostTime = now;
            return;
        }

        if (now - _lostTime > lostTimeout)
            Reset();

        return;
    }

    auto best = max_element(_hypotheses.begin(), _hypotheses.end(), [](const auto& a, const auto& b) {
        return a.confidence < b.confidence;
    });

    if (_hypotheses.size() == 1 && best->confidence >= 0.8) {
        _state = MappingEngineState::Stable;
        _stablePose = best->pose;
        return;
    }

    if (_hypotheses.size() == 1) {
        _state = MappingEngineState::Degraded;
        _stablePose = best->pose;
        return;
    }

    if (_state != MappingEngineState::Lost) {
        _state = MappingEngineState::Lost;
        _lostTime = now;
        return;
    }

    if (now - _lostTime > lostTimeout)
        Reset();
}

void MappingEngine::TryRelocalize()
{
    const auto currentContext = ComputeScanContext(_lastScan, _stablePose);
    const auto hash = currentContext.GetHash();

    const auto range = _scanContexts.equal_range(hash);
    for (auto context = range.first; context != range.second; ++context) {
        const auto similarity = ComputeScanContextSimilarity(currentContext, context->second);
        if (similarity <= 0.6)
            continue;

        const auto candidatePose = EstimatePoseFromScanContext(currentContext, context->second);

        _hypotheses.push_back(PoseMatchResult(candidatePose, similarity * 0.9));
    }
}

void MappingEngine::StoreScanContext()
{
    if (_state != MappingEngineState::Stable)
        return;

    constexpr auto maxScanContexts = 50;
    if (_scanContexts.size() > maxScanContexts) {
        auto it = _scanContexts.begin();

        for (size_t i = 0; i < maxScanContexts / 10 && it != _scanContexts.end(); ++i) {
            it = _scanContexts.erase(it);
        }
    }

    auto ctx = ComputeScanContext(_lastScan, _stablePose);
    _scanContexts.emplace(ctx.GetHash(), ctx);
}

ScanContext MappingEngine::ComputeScanContext(const std::vector<Vector2>& points, const Pose& pose)
{
    ScanContext context;
    context.pose = pose;

    for (auto& ring : context.descriptor)
        ring.fill(0.0f);

    constexpr auto maxRange = 10.0;
    constexpr auto ringStep = maxRange / ScanContext::ringCount;
    constexpr auto sectorStep = 2.0 * M_PI / ScanContext::sectorCount;

    for (const auto& p : points) {
        const double range = p.Magnitude();
        const double angle = std::atan2(p.y, p.x) + M_PI;

        const auto ring = std::min(static_cast<int>(range / ringStep), ScanContext::ringCount - 1);
        const auto sector = static_cast<int>(angle / sectorStep) % ScanContext::sectorCount;

        context.descriptor[ring][sector] = std::max(context.descriptor[ring][sector], static_cast<float>(range));
    }

    return context;
}

double MappingEngine::ComputeScanContextSimilarity(const ScanContext& a, const ScanContext& b)
{
    // Use cosine similarity averaged across all rings
    double totalSimilarity = 0.0;
    int validRings = 0;

    for (int r = 0; r < ScanContext::ringCount; ++r) {
        double dotProduct = 0.0;
        double normA = 0.0;
        double normB = 0.0;

        for (int s = 0; s < ScanContext::sectorCount; ++s) {
            dotProduct += a.descriptor[r][s] * b.descriptor[r][s];
            normA += a.descriptor[r][s] * a.descriptor[r][s];
            normB += b.descriptor[r][s] * b.descriptor[r][s];
        }

        if (normA > 0.0 && normB > 0.0) {
            totalSimilarity += dotProduct / (std::sqrt(normA) * std::sqrt(normB));
            ++validRings;
        }
    }

    return validRings > 0 ? totalSimilarity / validRings : 0.0;
}

Pose MappingEngine::EstimatePoseFromScanContext(const ScanContext& current, const ScanContext& reference)
{
    // Find best sector shift (rotation alignment)
    int bestShift = 0;
    double bestSimilarity = -1.0;

    for (int shift = 0; shift < ScanContext::sectorCount; ++shift) {
        double similarity = 0.0;

        for (int r = 0; r < ScanContext::ringCount; ++r) {
            for (int s = 0; s < ScanContext::sectorCount; ++s) {
                int shiftedS = (s + shift) % ScanContext::sectorCount;
                similarity += current.descriptor[r][s] * reference.descriptor[r][shiftedS];
            }
        }

        if (similarity > bestSimilarity) {
            bestSimilarity = similarity;
            bestShift = shift;
        }
    }

    // Each sector corresponds to (2*PI / sectorCount) radians
    constexpr double sectorAngle = 2.0 * M_PI / ScanContext::sectorCount;
    double rotationOffset = bestShift * sectorAngle;

    // The estimated pose is the reference pose with rotation adjusted
    Pose estimatedPose;
    estimatedPose.position = reference.pose.position;
    estimatedPose.rotation = reference.pose.rotation + rotationOffset;

    // Normalize rotation to [-PI, PI]
    while (estimatedPose.rotation > M_PI)
        estimatedPose.rotation -= 2.0 * M_PI;
    while (estimatedPose.rotation < -M_PI)
        estimatedPose.rotation += 2.0 * M_PI;

    return estimatedPose;
}

void MappingEngine::Reset()
{
    _state = MappingEngineState::Initializing;
    _hypotheses = { PoseMatchResult(Pose::Identity(), 0.5) };
    _scanContexts.clear();

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

    // PublishScan();
    PublishPose();
    PublishGrid();
    // PublishGridMap();
}

void MappingEngine::PublishScan() const
{
    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.stamp = _node->now();
    msg.header.frame_id = "map";

    msg.height = 1;
    msg.width = _lastScan.size();

    auto modifier = sensor_msgs::PointCloud2Modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(msg.width * msg.height);

    auto iterX = sensor_msgs::PointCloud2Iterator<float>(msg, "x");
    auto iterY = sensor_msgs::PointCloud2Iterator<float>(msg, "y");
    auto iterZ = sensor_msgs::PointCloud2Iterator<float>(msg, "z");

    for (const auto& p : _lastScan) {
        *iterX = static_cast<float>(p.x);
        *iterY = static_cast<float>(p.y);
        *iterZ = 0.0f;

        ++iterX;
        ++iterY;
        ++iterZ;
    }

    _scanPublisher->publish(msg);
}

void MappingEngine::PublishPose() const
{
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.stamp = _node->now();
    msg.header.frame_id = "map";

    for (const auto hypothesis : _hypotheses) {
        auto poseMsg = geometry_msgs::msg::Pose();
        poseMsg.position.x = hypothesis.pose.position.x;
        poseMsg.position.y = hypothesis.pose.position.y;
        poseMsg.position.z = 0.0;

        const double halfRotation = (hypothesis.pose.rotation + M_PI * 0.5) * 0.5;

        poseMsg.orientation.x = 0.0;
        poseMsg.orientation.y = 0.0;
        poseMsg.orientation.z = std::sin(halfRotation);
        poseMsg.orientation.w = std::cos(halfRotation);

        msg.poses.push_back(poseMsg);
    }

    // _path.poses.push_back(msg);
    // if (_path.poses.size() > 100)
    //     _path.poses.erase(_path.poses.begin());
    //
    // _pathPub->publish(_path);

    _posePublisher->publish(msg);
}

void MappingEngine::PublishGrid()
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
