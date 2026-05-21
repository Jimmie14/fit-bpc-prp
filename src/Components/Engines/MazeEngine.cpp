#include "Components/MazeEngine.hpp"

#include "Nav/PcaFilter.hpp"
#include "Viz/Color.hpp"
#include "Viz/Grid.hpp"
#include "Viz/Marker.hpp"

using namespace std;

namespace Manhattan::core {

using namespace Manhattan::nav;

// Junction detection
constexpr float HALF_CORRIDOR_SIZE = 0.2f;
constexpr float CORRIDOR_SIZE = 0.4f;

// RayArc settings
constexpr int RAY_COUNT = 9;
constexpr float RAY_DISTANCE = 5.0f;
constexpr float FOV = 20.0f;

// walls filtering
constexpr int MIN_POINTS_PER_SEGMENT = 4;

// Motor settings
constexpr float NORMAL_SPEED = 0.4f;
constexpr float TURN_SPEED = 0.0f;
constexpr float TURN_ANGULAR = 2.0f;

// PID tuning constants
constexpr float HEADING_P = 4.0f;
constexpr float HEADING_D = 0.05f;

constexpr float TURN_P = 3.5f;
constexpr float TURN_I = 0.005f;
constexpr float TURN_D = 0.01f;

static float NormalizeAngle(float angle)
{
    while (angle > M_PI)
        angle -= 2.f * M_PI;
    while (angle < -M_PI)
        angle += 2.f * M_PI;

    return angle;
}

static Vector2 ClosestPointOnLine(Vector2 direction, Vector2 start, Vector2 target)
{
    Vector2 normalizedDir = direction.normalized();
    Vector2 toTarget = target - start;
    auto projection = Vector2::dot(toTarget, normalizedDir);
    Vector2 closestPoint = start + normalizedDir * projection;

    return closestPoint;
}

MazeEngine::MazeEngine(const App& app)
    : RosEngine(app, "maze")
    , _headingPid(HEADING_P, 0.0f, HEADING_D)
    , _turnPid(TURN_P, TURN_I, TURN_D)
    , _previous_corridor_dir(1, 0)
{
    _mapping = app.getComponent<MappingEngine>();

    _publisher = create_publisher<visualization_msgs::msg::MarkerArray>("maze_walls", 1);
    _ledPublisher = create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/rgb_leds", 1);

    _app.events->Subscribe<CodeDetectedEvent>([this](const CodeDetectedEvent& event) {
        OnAruCode(event);
    });
}

void MazeEngine::OnEnable()
{
    _initialTimer = create_wall_timer(3s, [this] {
        _lastUpdate = std::chrono::steady_clock::now();
        _timer = create_wall_timer(10ms, [this] { Update(); });
        _initialTimer.reset();
    });

    _publisherTimer = create_wall_timer(100ms, [this] {
        publishDebug();
        publishLeds();
        // PublishHeading();
    });
}

void MazeEngine::OnDisable()
{
    _timer.reset();
    _initialTimer.reset();
    _publisherTimer.reset();
}

void MazeEngine::Update()
{
    auto now = std::chrono::steady_clock::now();
    _dt = duration<float>(now - _lastUpdate).count();
    _lastUpdate = now;

    switch (_state) {
    case NavState::FOLLOW_CORRIDOR:
        FollowCorridor();
        break;

    case NavState::TURNING:
        ExecuteTurnState();
        break;

    case NavState::RECENTER:
        RecenterState();
        break;

    default:
        break;
    }
}

void MazeEngine::FollowCorridor()
{
    _rays = {};

    CalculateWalls();

    PickDirection();
    if (_state != NavState::FOLLOW_CORRIDOR)
        return;

    float angular = 0.0f;
    const auto pose = _mapping->CurrentPose();

    if (_leftWall.has_value() && _rightWall.has_value()) {
        const auto targetAngle = GetHeadingError();
        angular = _headingPid.step(targetAngle, _dt);
    }

    auto frontDist = 0.0;
    for (auto hit : RayArc(5.0f, TurnDirection::FORWARD, RAY_DISTANCE)) {
        const auto dst = Vector2::distance(hit.hit, pose.position);
        frontDist += dst;
    }
    frontDist /= RAY_COUNT;

    float speed = std::clamp(static_cast<float>(frontDist) / CORRIDOR_SIZE, 0.0f, 1.0f);
    if (frontDist < 0.1f) speed = 0.0f;

    _app.events->Publish(MotorCommandEvent {
        Twist {
            NORMAL_SPEED * speed * speed,
            std::clamp(angular, -TURN_ANGULAR, TURN_ANGULAR) } });
}

bool MazeEngine::CalculateWalls()
{
    const auto leftHits = RayArc(FOV, TurnDirection::LEFT, RAY_DISTANCE);
    const auto rightHits = RayArc(FOV, TurnDirection::RIGHT, RAY_DISTANCE);

    auto leftWall = FilterHitPoints(leftHits);
    auto rightWall = FilterHitPoints(rightHits);

    if (!leftWall.has_value() || !rightWall.has_value()) return false;
    if (std::abs(Vector2::dot(leftWall.value().Direction, rightWall.value().Direction)) < 0.995) return false;

    const auto pose = _mapping->CurrentPose();
    if (Vector2::dot(leftWall->Direction, pose.forward()) < 0.0f)
        leftWall->Direction = -leftWall->Direction;

    if (Vector2::dot(rightWall->Direction, pose.forward()) < 0.0f)
        rightWall->Direction = -rightWall->Direction;

    const auto dir = (leftWall->Direction + rightWall->Direction).normalized();
    const auto dot = abs(Vector2::dot(dir, _previous_corridor_dir));
    if (dot < 0.7) return false;

    _leftWall = leftWall;
    _rightWall = rightWall;

    const auto closestPointOnLeft = ClosestPointOnLine(leftWall->Direction, leftWall->Point, pose.position);
    const auto dstOnLeft = Vector2::distance(closestPointOnLeft, pose.position);

    const auto closestPointOnRight = ClosestPointOnLine(rightWall->Direction, rightWall->Point, pose.position);
    const auto dstOnRight = Vector2::distance(closestPointOnRight, pose.position);

    constexpr auto wallDistance = 0.4f;

    const bool leftGreater = dstOnLeft > wallDistance;
    const bool rightGreater = dstOnRight > wallDistance;

    if (leftGreater && rightGreater) {
        _leftWall->Point = (closestPointOnLeft - pose.position).normalized() * (wallDistance * 0.5f) + pose.position;
        _rightWall->Point = (closestPointOnRight - pose.position).normalized() * (wallDistance * 0.5f) + pose.position;

        return true;
    }

    if (dstOnLeft > wallDistance)
        _leftWall->Point = (closestPointOnLeft - pose.position).normalized() * (wallDistance - dstOnRight) + pose.position;

    if (dstOnRight > wallDistance)
        _rightWall->Point = (closestPointOnRight - pose.position).normalized() * (wallDistance - dstOnLeft) + pose.position;

    return true;
}

void MazeEngine::PickDirection()
{
    const bool openLeft = DirectionIsFree(TurnDirection::LEFT);
    const bool openRight = DirectionIsFree(TurnDirection::RIGHT);
    const bool openFront = DirectionIsFree(TurnDirection::FORWARD, openLeft || openRight ? HALF_CORRIDOR_SIZE * 1.5 : HALF_CORRIDOR_SIZE * 0.9);

    if (!(openLeft == _previous_left && openRight == _previous_right && openFront == _previous_front)) {
        _previous_left = openLeft;
        _previous_right = openRight;
        _previous_front = openFront;
        return;
    }
    _previous_left = openLeft;
    _previous_right = openRight;
    _previous_front = openFront;

    // std::cout << "Front: " << openFront << " Left: " << openLeft << " Right: " << openRight << std::endl;
    // return;
    const auto now = std::chrono::steady_clock::now();
    if (now - _lastTurn < 2s)
        return;

    const auto pose = _mapping->CurrentPose();

    switch (ChooseDirection(openLeft, openFront, openRight)) {
    case TurnDirection::LEFT:
        _targetRotation = static_cast<float>(pose.theta + M_PI_2);
        // std::cout << "Turning left" << std::endl;
        break;

    case TurnDirection::RIGHT:
        _targetRotation = static_cast<float>(pose.theta - M_PI_2);
        // std::cout << "Turning right" << std::endl;
        break;

    case TurnDirection::FORWARD:
        _targetRotation = static_cast<float>(pose.theta);
        // std::cout << "Going forward" << std::endl;
        _state = NavState::FOLLOW_CORRIDOR;
        return;
    case TurnDirection::BACK:
        _targetRotation = static_cast<float>(pose.theta - M_PI);
        // std::cout << "Turning back" << std::endl;
        break;
    }

    _previous_corridor_dir = Vector2::FromAngle(_targetRotation);
    _state = NavState::TURNING;
    _targetRotation = NormalizeAngle(_targetRotation);
}

void MazeEngine::ExecuteTurnState()
{
    const auto pose = _mapping->CurrentPose();
    const float error = NormalizeAngle(_targetRotation - static_cast<float>(pose.theta));

    _lastTurn = std::chrono::steady_clock::now();
    ;

    if (std::abs(error) < 0.01f) {
        _state = NavState::FOLLOW_CORRIDOR;
        _turnPid.reset();
        return;
    }

    const float angular = _turnPid.step(error, _dt);
    _app.events->Publish(MotorCommandEvent {
        Twist {
            TURN_SPEED,
            std::clamp(angular, -TURN_ANGULAR, TURN_ANGULAR) } });
}

void MazeEngine::RecenterState()
{
    const auto pose = _mapping->CurrentPose();
    const float error = NormalizeAngle(_targetRotation - static_cast<float>(pose.theta));

    if (std::abs(error) < 0.08f) {
        _state = NavState::FOLLOW_CORRIDOR;
        _turnPid.reset();

        return;
    }

    const float angular = _turnPid.step(error, _dt);
    _app.events->Publish(MotorCommandEvent {
        Twist {
            TURN_SPEED,
            std::clamp(angular, -TURN_ANGULAR, TURN_ANGULAR) } });
}

bool MazeEngine::DirectionIsFree(const TurnDirection side, float frontDstOverride)
{
    // const auto pose = _mapping->CurrentPose();
    auto offset = HALF_CORRIDOR_SIZE * 1.1f;

    auto center = _center;
    auto direction = _heading;

    switch (side) {
    case TurnDirection::FORWARD: {
        RayHit rayHit;

        const auto hit = _mapping->RayCast(center, direction, rayHit, frontDstOverride);

        _rays.emplace_back(center, direction, hit);
        return !hit;
    }
    case TurnDirection::LEFT:
        direction = -Vector2::Perpendicular(direction);
        break;
    case TurnDirection::RIGHT:
        direction = Vector2::Perpendicular(direction);
        break;
    case TurnDirection::BACK:
        direction = -direction;
        break;
    }

    auto perpendicularDir = Vector2::Perpendicular(direction);

    float frontDst = offset;
    if (RayHit rayHit; _mapping->RayCast(center, perpendicularDir, rayHit, offset + 0.05f)) {
        frontDst = static_cast<float>(Vector2::distance(center, rayHit.hit));
    }

    float backDst = offset;
    if (RayHit rayHit; _mapping->RayCast(center, -perpendicularDir, rayHit, offset + 0.05f)) {
        backDst = static_cast<float>(Vector2::distance(center, rayHit.hit));
    }

    // dst to front + dst in back
    auto stepSize = (frontDst + backDst) / (RAY_COUNT * 2 - 1);
    auto origin = center - perpendicularDir * backDst;

    bool hasFirstHit = false;
    int firstHit = -1;
    int lastHit = -1;

    for (auto i = 0; i < RAY_COUNT * 2; i++) {
        const auto point = origin + perpendicularDir * stepSize * i;

        if (RayHit rayHit; _mapping->RayCast(point, direction, rayHit, CORRIDOR_SIZE)) {
            if (!hasFirstHit)
                firstHit = i;
            else if (lastHit < 0)
                lastHit = i;

            _rays.push_back({ point, direction, true });
        } else {
            if (firstHit >= 0)
                hasFirstHit = true;

            _rays.push_back({ point, direction, false });
        }
    }

    return firstHit >= 0 && lastHit >= 0;
}

float MazeEngine::GetHeadingError()
{
    const auto pose = _mapping->CurrentPose();
    auto leftDir = _leftWall->Direction.normalized();
    auto rightDir = _rightWall->Direction.normalized();

    if (Vector2::dot(leftDir, rightDir) < 0.0f)
        rightDir = -rightDir;

    const Vector2 corridorDir = (leftDir + rightDir).normalized();

    const auto leftPoint = ClosestPointOnLine(leftDir, _leftWall->Point, pose.position);
    const auto rightPoint = ClosestPointOnLine(rightDir, _rightWall->Point, pose.position);

    const auto center = (leftPoint + rightPoint) * 0.5f;

    _heading = corridorDir.normalized();
    _center = center;

    const auto target = center + corridorDir * 0.3;
    return Vector2::signedAngle(pose.forward(), (target - pose.position).normalized());
}

std::vector<RayHit> MazeEngine::RayArc(const float fov, const TurnDirection side, const float dst) const
{
    const auto pose = _mapping->CurrentPose();
    float baseAngle = 0;
    switch (side) {
    case TurnDirection::FORWARD:
        baseAngle = static_cast<float>(pose.theta);
        break;
    case TurnDirection::LEFT:
        baseAngle = static_cast<float>(pose.theta + M_PI_2);
        break;
    case TurnDirection::RIGHT:
        baseAngle = static_cast<float>(pose.theta - M_PI_2);
        break;
    case TurnDirection::BACK:
        baseAngle = static_cast<float>(pose.theta - M_PI);
        break;
    }

    const auto rad = fov * M_PI / 180.0;
    const auto angleStep = rad / (RAY_COUNT - 1);

    auto angle = baseAngle - rad * 0.5;

    std::vector<RayHit> hits;

    for (int i = 0; i <= RAY_COUNT; ++i) {
        RayHit ray;
        const auto hit = _mapping->RayCast(pose.position, Vector2::FromAngle(angle), ray, dst);
        angle += angleStep;

        if (!hit) continue;
        hits.push_back(ray);
    }

    return hits;
}

std::optional<PcaFitter::FittedLine> MazeEngine::FilterHitPoints(const std::vector<RayHit>& hits) const
{
    if (hits.empty())
        return std::nullopt;

    std::vector<PcaFitter::FittedLine> lines;
    std::vector<Vector2> points;

    // Build contiguous segments
    points.push_back(hits[0].hit);

    for (size_t i = 1; i < hits.size(); ++i) {
        const auto prev = hits[i - 1];
        const auto curr = hits[i];

        const auto dot = Vector2::dot(prev.normal, curr.normal);

        if (dot < 0.8f) {
            if (points.size() >= MIN_POINTS_PER_SEGMENT) {
                auto line = PcaFitter::FitLine(points);
                line.PointCount = static_cast<int>(points.size());

                lines.push_back(line);
            }

            points.clear();
        }

        points.push_back(curr.hit);
    }

    // Add final segment
    if (points.size() >= MIN_POINTS_PER_SEGMENT) {
        auto line = PcaFitter::FitLine(points);
        line.PointCount = static_cast<int>(points.size());

        lines.push_back(line);
    }

    // No valid segments
    if (lines.empty())
        return std::nullopt;

    if (lines.size() == 1)
        return lines[0];

    const auto pose = _mapping->CurrentPose();
    const Vector2 robotPos = pose.position;
    const Vector2 forward = pose.forward();

    std::optional<PcaFitter::FittedLine> bestLine;
    auto bestScore = -std::numeric_limits<double>::infinity();

    for (const auto& line : lines) {
        const auto alignment = std::abs(Vector2::dot(line.Direction.normalized(), forward));

        Vector2 toLine(
            line.Point.x - robotPos.x,
            line.Point.y - robotPos.y);

        Vector2 lineNormal = line.Normal;

        const auto distToLine = std::abs(Vector2::dot(toLine, lineNormal));

        // Reject very far walls
        if (distToLine > 3.0f)
            continue;

        // Reject badly aligned walls
        if (alignment < 0.5f)
            continue;

        auto score = 0.0;

        // Prefer parallel walls
        score += alignment * 5.0f;

        // Prefer closer walls
        score -= distToLine * 2.0f;

        // Prefer larger segments
        score += static_cast<float>(line.PointCount) * 0.5f;

        if (score > bestScore) {
            bestScore = score;
            bestLine = line;
        }
    }

    return bestLine;
}

TurnDirection MazeEngine::ChooseDirection(const bool left, const bool forward, const bool right)
{
    const bool cornerLeft = left && !right && !forward;
    const bool cornerRight = right && !left && !forward;
    const bool corner = cornerLeft || cornerRight;
    const bool onlyFront = forward && !left && !right;
    const bool deadEnd = !forward && !left && !right;

    if (!_waypoints.empty() && !corner && !onlyFront && !deadEnd) {
        std::cout << "Aru? Front: " << forward << " Left: " << left << " Right: " << right << std::endl;

        std::lock_guard lock(_mutex);
        const auto waypoint = _waypoints.front();
        auto aruCode = waypoint.treasureCode.has_value() ? waypoint.treasureCode.value() : waypoint.exitCode.value();

        _lastTurn = std::chrono::steady_clock::now();
        _waypoints.erase(_waypoints.begin());

        switch (aruCode.id % 10) {
        case 0:
            return forward ? TurnDirection::FORWARD : TurnDirection::BACK;
        case 1:
            return left ? TurnDirection::LEFT : TurnDirection::RIGHT;
        case 2:
            return right ? TurnDirection::RIGHT : TurnDirection::LEFT;
        default:
            return TurnDirection::LEFT;
        }
    }

    if (right) return TurnDirection::RIGHT;
    if (forward) return TurnDirection::FORWARD;
    if (left) return TurnDirection::LEFT;

    return TurnDirection::BACK;
}

void MazeEngine::OnAruCode(CodeDetectedEvent aruCode)
{
    std::lock_guard lock(_mutex);

    // std::cout << "AruCode detected: " << aruCode.id << std::endl;

    if (_waypoints.empty())
        _waypoints.push_back({ std::nullopt, std::nullopt });

    auto& waypoint = _waypoints.back();
    if (aruCode.id >= 10) {
        if (waypoint.treasureCode.has_value() && waypoint.treasureCode->id != aruCode.id)
            _waypoints.push_back({ std::nullopt, aruCode });
        else
            waypoint.treasureCode = aruCode;
    } else {
        if (waypoint.exitCode.has_value() && waypoint.exitCode->id != aruCode.id)
            _waypoints.push_back({ aruCode, std::nullopt });
        else
            waypoint.exitCode = aruCode;
    }

    // std::cout << "Count " << _waypoints.size() << std::endl;
}

void MazeEngine::publishDebug() const
{
    auto markers = viz::marker::MarkerArrayBuilder();

    markers.add(viz::marker::clear("map"));

    markers.color = viz::color::cyan;
    if (_leftWall.has_value()) {
        const auto p1 = _leftWall->Point - _leftWall->Direction * 2.0;
        const auto p2 = _leftWall->Point + _leftWall->Direction * 2.0;

        markers.add(viz::marker::path({ p1, p2 }, "map"));
    }

    if (_rightWall.has_value()) {
        const auto p1 = _rightWall->Point - _rightWall->Direction * 2.0;
        const auto p2 = _rightWall->Point + _rightWall->Direction * 2.0;

        markers.add(viz::marker::line(p1, p2, "map"));
    }

    for (auto& [origin, direction, hit] : _rays) {
        markers.color = hit ? viz::color::red : viz::color::green;
        markers.add(viz::marker::line(origin, origin + direction * HALF_CORRIDOR_SIZE, "map"));
    }

    _publisher->publish(markers.array);
}

void MazeEngine::publishLeds() const
{
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(12);

    if (_waypoints.empty()) {
        _ledPublisher->publish(msg);
        return;
    }

    auto waypoint = _waypoints.front();

    // 0 - back
    // 1 - right
    // 2 - left
    // 3 - front

    if (waypoint.treasureCode.has_value()) {
        switch (waypoint.treasureCode->id % 10) {
        case 0:
            msg.data[3 * 3 + 1] = 255;
            break;
        case 1:
            msg.data[2 * 3 + 1] = 255;
            break;
        case 2:
            msg.data[1 * 3 + 1] = 255;
            break;
        default:
            break;
        }

        _ledPublisher->publish(msg);
        return;
    }

    if (!waypoint.exitCode.has_value()) {
        _ledPublisher->publish(msg);
    }

    switch (waypoint.treasureCode->id) {
    case 0:
        msg.data[3 * 3 + 1] = 255;
        msg.data[3 * 3 + 2] = 255;
        msg.data[3 * 3 + 3] = 255;
        break;
    case 1:
        msg.data[2 * 3 + 1] = 255;
        msg.data[2 * 3 + 2] = 255;
        msg.data[2 * 3 + 3] = 255;
        break;
    case 2:
        msg.data[1 * 3 + 1] = 255;
        msg.data[1 * 3 + 2] = 255;
        msg.data[1 * 3 + 3] = 255;
        break;
    default:
        break;
    }

    _ledPublisher->publish(msg);
}

} // namespace Manhattan::Core