#include "Components/MappingEngine.hpp"
#include "Components/LidarDriver.hpp"
#include "Math/Vec3.hpp"
#include "Messages/Nav.hpp"
#include "Messages/RobotMode.hpp"
#include "Viz/Marker.hpp"
#include "Viz/Grid.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <random>

using namespace std;

constexpr auto gridResolution = 0.05;
constexpr auto rotationResolution = M_PI * 0.25;

const float maxInvalidError = 0.4f;
const float poseChangeThreshold = 0.10f;
const float stablePoseError = 0.1f;

namespace Manhattan::core {

using namespace Manhattan::messages;

MappingEngine::MappingEngine(const App& app)
    : RosEngine(app, "mapping")
    , _grid(Vector2i(200, 200), gridResolution, 8, 20)
    , _distanceField(200, 200, gridResolution)
    , _poseMatcher(PoseMatcher(_grid, 5))
    , _lastOdomPose(Pose::zero())
    , _activeHypothesis(PoseMatchResult(Pose::zero(), maxInvalidError - 0.1))
    , _hypotheses({})
{
    _lostTime = now();

    app.events->Subscribe<LidarScan>([this](const LidarScan& scan) {
        this->OnLidar(scan.points);
    });

    app.events->Subscribe<RobotResetEvent>([this](const RobotResetEvent& _) {
        this->Reset();
    });

    app.events->Subscribe<RobotModeChangeEvent>([this](const RobotModeChangeEvent& event) {
        _reverse = event.newMode.reverse;
    });

    _posePublisher = create_publisher<geometry_msgs::msg::PoseStamped>("slam/pose", 1);
    _pathPublisher = create_publisher<nav_msgs::msg::Path>("slam/path", 1);

    _debugPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("slam/debug", 1);

    _scanPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("slam/scan", 1);
    _gridPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("slam/grid", 1);
    _distanceFieldPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("slam/distance_field", 1);
    _gridMapPublisher = create_publisher<grid_map_msgs::msg::GridMap>("slam/grid_map", 1);

    _path.header.frame_id = "map";

    _odometrySubscription = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->OnOdometry(msg);
        });

    _poseUpdateTimer = create_wall_timer(10ms, [this] {
        PublishStablePose();
    });

    _publishTimer = create_wall_timer(200ms, [this] {
        Publish();
    });

    _costUpdateTimer = create_wall_timer(1000ms, [this] {
        std::lock_guard guard(_mapLock);

        this->_grid.RecalculateCosts();
    });
}

void MappingEngine::OnOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    const auto current = Pose::fromRosPoseMessage(msg->pose.pose);

    {
        std::lock_guard guard(_odomLock);

        const auto delta = current - _lastOdomPose;

        _odomPoseDelta = _odomPoseDelta + delta;

        _lastOdomPose = current;
    }

    _twist = Twist::fromRosTwistMessage(msg->twist.twist);
}

void MappingEngine::OnLidar(const vector<Vector2>& points)
{
    if (points.empty()) return;

    Pose odomDelta;
    {
        std::lock_guard guard(_odomLock);

        odomDelta = _odomPoseDelta;
        _odomPoseDelta = Pose::zero();
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
        const auto stableResult = MatchPose(hypothesis.pose);
        const auto odomResult = MatchPose(hypothesis.pose + odomDelta);

        hypothesis = PoseMatchResult::Best(stableResult, odomResult);
    }

    const auto stableResult = MatchPose(_activeHypothesis.pose);
    const auto odomResult = MatchPose(_activeHypothesis.pose + odomDelta);

    _activeHypothesis = PoseMatchResult::Best(stableResult, odomResult);

    erase_if(_hypotheses, [](const PoseMatchResult& result) {
        return result.error > maxInvalidError;
    });

    auto best = ranges::min_element(_hypotheses, [](const auto& a, const auto& b) {
        return a.error < b.error;
    });

    if (best != _hypotheses.end() && best->error + poseChangeThreshold < _activeHypothesis.error) {
        _hypotheses.push_back(_activeHypothesis);
        _activeHypothesis = *best;

        _hypotheses.erase(best);
    }

    if (_state == MappingEngineState::Degraded || _hypotheses.size() < 5)
        CreateHypothesis();

    UpdateState();
}

void MappingEngine::UpdateState()
{
    constexpr auto lostTimeout = 10s;

    const auto timeNow = now();

    if (_hypotheses.empty()) {
        if (_state != MappingEngineState::Lost) {
            ChangeState(MappingEngineState::Lost);
            _lostTime = timeNow;
            return;
        }

        if (timeNow - _lostTime > lostTimeout)
            Reset();

        return;
    }

    if (_activeHypothesis.error < maxInvalidError) {
        _stablePose = _activeHypothesis.pose;
    }

    if (_activeHypothesis.error < stablePoseError) {
        ChangeState(MappingEngineState::Stable);
        return;
    }

    if (_hypotheses.size() < 2) {
        ChangeState(MappingEngineState::Degraded);
        return;
    }

    if (_state != MappingEngineState::Lost) {
        ChangeState(MappingEngineState::Lost);
        _lostTime = timeNow;
        return;
    }

    if (timeNow - _lostTime > lostTimeout)
        Reset();
}

void MappingEngine::ChangeState(const MappingEngineState newState)
{
    if (_state == newState)
        return;

    const auto oldState = _state;

    _state = newState;

    _app.events->Publish(MappingEngineStateChangeEvent {
        .oldState = oldState,
        .newState = newState
    });

    if (newState == MappingEngineState::Lost) {
        _app.events->Publish(RobotEnvironmentChangeEvent { });
    }

    if (oldState == MappingEngineState::Lost && newState == MappingEngineState::Stable) {
        _app.events->Publish(RobotEnvironmentChangeEvent { });
    }
}

void MappingEngine::CreateHypothesis()
{
    if (_hypotheses.empty()) {
        _hypotheses = { _activeHypothesis };
        return;
    }

    const auto it = ranges::min_element(_hypotheses, [](const auto& a, const auto& b) {
        return a.error < b.error;
    });

    if (it == _hypotheses.end())
        return;

    auto best = *it;

    static thread_local auto prng = std::mt19937(std::random_device {}());
    constexpr auto attempts = 30;

    auto offsetXY = std::uniform_real_distribution(-0.75, 0.75);
    auto offsetTheta = std::uniform_real_distribution(-M_PI, M_PI);

    for (auto i = 0; i < attempts; i++) {
        const auto candidatePose = Pose(
            Vector2(best.pose.position.x + offsetXY(prng), best.pose.position.y + offsetXY(prng)),
            best.pose.theta + offsetTheta(prng));

        const auto hypothesis = MatchPose(candidatePose);
        if (hypothesis.error > maxInvalidError) continue;

        _hypotheses.push_back(hypothesis);

        best = PoseMatchResult::Best(best, hypothesis);
    }
}

void ClearSimilarHypotheses(std::vector<PoseMatchResult>& hypotheses)
{
    using Key = std::tuple<int, int, int>;
    auto keyHash = [](const Key& k) -> size_t {
        auto [a, b, c] = k;
        return std::hash<int>()(a) ^ std::hash<int>()(b) ^ std::hash<int>()(c);
    };

    auto seen = std::unordered_set<std::tuple<int, int, int>, decltype(keyHash)>(0, keyHash);
    seen.reserve(hypotheses.size());

    size_t write = 0;

    for (size_t read = 0; read < hypotheses.size(); read++) {
        const auto p = hypotheses[read].pose.normalized();

        const auto key = std::tuple<int, int, int>(static_cast<int>(std::round(p.position.x / gridResolution)),
            static_cast<int>(std::round(p.position.y / gridResolution)),
            static_cast<int>(std::round(p.theta / rotationResolution)));
    }
}

void MappingEngine::Reset()
{
    _hypotheses = { PoseMatchResult(Pose::zero(), maxInvalidError - 0.1) };
    _grid.Reset();

    _stablePose = Pose::zero();

    ChangeState(MappingEngineState::Initializing);
}

static double estimateInitialTheta(const std::vector<Vector2>& points)
{
    vector<double> angles;
    angles.reserve(points.size() - 1);

    for (auto i = 1; i < points.size(); i++) {
        const auto& previous = points[i - 1];
        const auto& current = points[i];

        angles.emplace_back(Vector2::signedAngle(current - previous, Vector2(1, 0)));
    }

    ranges::sort(angles);

    double best = angles[0];
    int bestCount = 1;

    double current = angles[0];
    int currentCount = 1;

    for (size_t i = 1; i < angles.size(); ++i) {
        if (std::fabs(angles[i] - current) <= 0.05) {
            currentCount++;
            continue;

        }

        if (currentCount > bestCount) {
            bestCount = currentCount;
            best = current;
        }

        current = angles[i];
        currentCount = 1;
    }

    if (currentCount > bestCount) {
        best = current;
    }

    return best;
}

void MappingEngine::MapScan(const std::vector<Vector2>& points)
{
    if (_state != MappingEngineState::Stable && _state != MappingEngineState::Initializing)
        return;

    if (_state == MappingEngineState::Initializing) {
        const auto theta = estimateInitialTheta(points);

        _stablePose.theta = theta;
        _activeHypothesis.pose.theta = theta;

        ChangeState(MappingEngineState::Lost);
    }

    // Convert robot position to grid coordinates
    const auto startGridPos = _grid.WorldToGrid(_stablePose.position);

    const auto cosRot = std::cos(_stablePose.theta);
    const auto sinRot = std::sin(_stablePose.theta);

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
            if (_grid.GetCell(cell)->IsOccupied()) break;
        }

        // Mark the endpoint as occupied
        _grid.SetOccupied(endGridPos);
    }

    RecalculateDistanceField();
}

void MappingEngine::RecalculateDistanceField()
{
    const auto defaultValue = 200.0 * 200.0 * _distanceField.resolution();

    vector<Vector2i> obstacles;

    for (auto cell : _grid) {
        if (cell.IsUnknown()) continue;
        if (!cell.IsOccupied()) continue;

        obstacles.push_back(cell.GetGridPosition());
    }

    for (auto i = 0; i < _distanceField.size(); i++) {
        const auto pos = Vector2i(_distanceField.indexToCoord(i));

        auto min = defaultValue;

        for (auto obstacle : obstacles) {
            const auto dist = Vector2(obstacle - pos).magnitude() * _distanceField.resolution();

            min = std::min(min, dist);
        }

        _distanceField.set(i, min);
    }
}

PoseMatchResult MappingEngine::MatchPose(const Pose& pose) const
{
    auto best = PoseMatchResult(pose, maxInvalidError);

    for (auto i = 0; i < 16; i++) {
        const auto correction = (static_cast<double>(i) / 16.0) * 2 * M_PI;

        auto newPose = pose;
        newPose.theta += correction;
        newPose = newPose.normalized();

        auto result = PoseMatchResult(newPose, GetDistanceFieldError(newPose));

        best = PoseMatchResult::Best(best, result);
    }

    auto result = _poseMatcher.Match(_lastScan, best.pose);
    result.error = GetDistanceFieldError(result.pose);

    return result;
}

double MappingEngine::GetDistanceFieldError(const Pose& pose) const
{
    auto error = 0.0;

    for (auto point : pose.transformPoints(_lastScan)) {
        const auto pos = _grid.WorldToGrid(point);
        if (!_grid.InBounds(pos.x, pos.y)) {
            error += 200.0 * 200.0 * _distanceField.resolution();
            continue;
        }

        error += _distanceField[pos];
    }

    error /= static_cast<double>(_lastScan.size());

    return error;
}

void MappingEngine::Publish()
{
    std::lock_guard lock(_mapLock);

    PublishDebug();
    PublishPose();
    PublishGrid();
    PublishDistanceField();
    // PublishGridMap();
    PublishScan();


}

void MappingEngine::PublishStablePose() const
{
    _app.events->Publish(RobotPoseEvent {
        .pose = _stablePose.normalized(),
        .twist = _twist
    });
}

void MappingEngine::PublishScan() const
{
    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.stamp = now();
    msg.header.frame_id = "map";

    msg.height = 1;
    msg.width = _lastScan.size();

    auto modifier = sensor_msgs::PointCloud2Modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(msg.width * msg.height);

    auto iterX = sensor_msgs::PointCloud2Iterator<float>(msg, "x");
    auto iterY = sensor_msgs::PointCloud2Iterator<float>(msg, "y");
    auto iterZ = sensor_msgs::PointCloud2Iterator<float>(msg, "z");

    for (const auto& p : _stablePose.transformPoints(_lastScan)) {
        *iterX = static_cast<float>(p.x);
        *iterY = static_cast<float>(p.y);
        *iterZ = 0.0f;

        ++iterX;
        ++iterY;
        ++iterZ;
    }

    _scanPublisher->publish(msg);
}

void MappingEngine::PublishDebug() const
{
    auto markers = viz::marker::MarkerArrayBuilder();

    markers.add(viz::marker::clear("map"));

    for (const auto hypothesis : _hypotheses) {
        markers.color = viz::color::red;
        markers.scale = 0.5f;

        const auto start = hypothesis.pose.position.toTf2();
        markers.add(viz::marker::direction(start, hypothesis.pose.forward().toTf2(), "map"));


        markers.color = viz::color::black;
        markers.scale = 0.25f;

        const auto textPos = start + vec3::Up * 0.02f;
        markers.add(viz::marker::text(textPos, std::to_string(hypothesis.error), "map"));
    }

    _debugPublisher->publish(markers.array);
}

void MappingEngine::PublishPose()
{
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
    gridMsg.info.origin.position.y = _grid.GetHeight() * _grid.GetCellSize() * -0.5;
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

            gridMsg.data[_grid.GetIndex(x, y)] = cell->IsUnknown() ? -1 : static_cast<int8_t>(cell->GetProbability() * 100.0);
        }
    }

    _gridPublisher->publish(gridMsg);
}

void MappingEngine::PublishDistanceField() const
{
    const auto msg = viz::nav::ToDistanceFieldMessage(_distanceField, "map");

    _distanceFieldPublisher->publish(msg);
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

GridCell* MappingEngine::GetCell(Vector2i gridPosition)
{
    return _grid.GetCell(gridPosition);
}

Vector2 MappingEngine::GridToWorld(const Vector2i& pos) const
{
    return _grid.GridToWorld(pos);
}

Vector2i MappingEngine::WorldToGrid(const Vector2& pos) const
{
    return _grid.WorldToGrid(pos);
}

std::vector<GridCell*> MappingEngine::GetNeighbors(const GridCell* cell)
{
    auto neighbours = std::vector<GridCell*>();

    for (auto direction : Vector2i::directions()) {
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
    const auto endWorldPosition = worldPosition + direction.normalized() * maxDistance;
    const auto endCell = _grid.WorldToGrid(endWorldPosition);

    rayHit = RayHit();

    for (const auto& pos : OccupancyGrid::Bresenham(startCell, endCell)) {
        const auto cell = _grid.GetCell(pos);
        if (cell == nullptr || !cell->IsOccupied())
            continue;

        rayHit.hit = _grid.GridToWorld(pos);

        auto normalInt = Vector2i::zero();

        for (auto dir : Vector2i::directions()) {
            const auto nCell = _grid.GetCell(pos + dir);

            if (nCell != nullptr && nCell->IsOccupied())
                continue;

            normalInt = normalInt + dir;
        }

        if (normalInt != Vector2i::zero()) {
            rayHit.normal = Vector2(normalInt).normalized();

            if (Vector2::dot(rayHit.normal, direction) > 0.0)
                rayHit.normal = -rayHit.normal;

            return true;
        }

        const auto toStart = Vector2(startCell - pos);

        rayHit.normal = toStart.sqrMagnitude() > 0.0 ? toStart.normalized() : -direction.normalized();

        return true;
    }

    return false;
}
} // namespace Manhattan::Core
