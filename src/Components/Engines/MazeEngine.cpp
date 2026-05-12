#include "MazeEngine.hpp"

#include "Nav/PcaFilter.hpp"
#include "Viz/Grid.hpp"

using namespace std;

namespace Manhattan::Core {

using namespace Manhattan::nav;

// Junction detection
constexpr float OPEN_THRESHOLD = 0.4f;

// RayArc settings
constexpr int RAY_COUNT = 9;
constexpr float RAY_DISTANCE = 1.0f;
constexpr float FORWARD_RAY_DISTANCE = 5.0f;
constexpr float FOV = 160.0f;
constexpr float FORWARD_FOV = 10.0f;

// walls filtering
constexpr int MIN_POINTS_PER_SEGMENT = 2;

// Motor settings
constexpr float NORMAL_SPEED = 0.015f;
constexpr float TURN_SPEED = 0.0f;
constexpr float TURN_ANGULAR = 0.5f;

// PID tuning constants
constexpr float HEADING_P = 0.1f;
constexpr float CENTER_P = 0.4f;
constexpr float HEADING_D = 0.01f;
constexpr float CENTER_D = 0.01f;

constexpr float TURN_P = 0.4f;
constexpr float TURN_D = 0.01f;

static float NormalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.f * M_PI;
    while (angle < -M_PI) angle += 2.f * M_PI;

    return angle;
}

static Vector2 ClosestPointOnLine(Vector2 direction, Vector2 start, Vector2 target)
{
    Vector2 normalizedDir = direction.Normalized();
    Vector2 toTarget = target - start;
    float projection = Vector2::Dot(toTarget, normalizedDir);
    Vector2 closestPoint = start + normalizedDir * projection;

    return closestPoint;
}

MazeEngine::MazeEngine(const App& app)
    : RosEngine(app, "maze")
{
    _mapping = app.GetComponent<MappingEngine>();

    _publisher = create_publisher<visualization_msgs::msg::MarkerArray>("maze_walls", 1);

    _app.Events->Subscribe<CodeDetectedEvent>([this](const CodeDetectedEvent& event) {
        OnAruCode(event);
    });
}

void MazeEngine::OnEnable() {
    _initialTimer = create_wall_timer(3s, [this] {
        _lastUpdate = std::chrono::steady_clock::now();
        _timer = create_wall_timer(10ms, [this] { Update(); });
        _initialTimer.reset();
    });

    _publisherTimer = create_wall_timer(100ms, [this] {
        PublishWalls();
    });
}

void MazeEngine::OnDisable() {
    _timer.reset();
    _initialTimer.reset();
    _publisherTimer.reset();
}

void MazeEngine::Update() {
    auto now = std::chrono::steady_clock::now();
    _dt = duration<float>(now - _lastUpdate).count();
    _lastUpdate = now;

    switch (_state)
    {
        case NavState::FOLLOW_CORRIDOR:
                FollowCorridor();
                break;

            case NavState::TURNING:
                ExecuteTurn();
                break;

            default:
                break;
    }
}

void MazeEngine::FollowCorridor()
{
    const float leftDist = GetWallDistance(TurnDirection::LEFT);
    const float rightDist = GetWallDistance(TurnDirection::RIGHT);
    const float frontDist = GetWallDistance(TurnDirection::FORWARD);

    const bool openLeft = leftDist > OPEN_THRESHOLD;
    const bool openRight = rightDist > OPEN_THRESHOLD;
    const bool openFront = frontDist > OPEN_THRESHOLD;

    const bool tJunction = openLeft && openRight && !openFront;
    const bool cornerLeft = openLeft && !openRight && !openFront;
    const bool cornerRight = openRight && !openLeft && !openLeft;
    const bool xJunction = openLeft && openRight && openFront;
    const bool deadEnd = !openLeft && !openRight && !openFront;

    if (tJunction || cornerLeft || cornerRight || xJunction || deadEnd)
    {
        StartDecision(openLeft, openFront, openRight);

        if (_state != NavState::FOLLOW_CORRIDOR)
            return;
    }

    CalculateWalls();

    float centerError = 0.0f;
    float headingError = 0.0f;

    float centerDerivative = 0.0f;
    float headingDerivative = 0.0f;

    if (_leftWall.has_value() && _rightWall.has_value()){
        const auto pose = _mapping->CurrentPose();

        Vector2 leftDir  = _leftWall->Direction.Normalized();
        Vector2 rightDir = _rightWall->Direction.Normalized();

        if (Vector2::Dot(leftDir, rightDir) < 0.0f)
            rightDir = -rightDir;

        const Vector2 corridorDir =
            (leftDir + rightDir).Normalized();

        Vector2 corridorNormal =
            Vector2::Perpendicular(corridorDir).Normalized();

        const Vector2 leftToRight =
            _rightWall->Point - _leftWall->Point;

        if (Vector2::Dot(corridorNormal, leftToRight) < 0.0f)
            corridorNormal = -corridorNormal;

        headingError =
            Vector2::Cross(pose.forward, corridorDir);

        headingDerivative =
            (headingError - _prevHeadingError) / _dt;

        _prevHeadingError = headingError;

        const Vector2 leftClosest =
            ClosestPointOnLine(
                leftDir,
                _leftWall->Point,
                pose.position);

        const Vector2 rightClosest =
            ClosestPointOnLine(
                rightDir,
                _rightWall->Point,
                pose.position);

        const Vector2 centerPoint =
            (leftClosest + rightClosest) * 0.5f;

        const Vector2 errorVec =
            pose.position - centerPoint;

        centerError =
            Vector2::Dot(errorVec, corridorNormal);

        centerDerivative =
            (centerError - _prevCenterError) / _dt;

        _prevCenterError = centerError;

        std::cout
            << "center: " << centerError
            << " heading: " << headingError
            << std::endl;
    }

    const float speed = std::clamp(frontDist / OPEN_THRESHOLD, 0.0f, 1.0f);
    const float angular =
        HEADING_P * headingError +
        CENTER_P  * centerError +
        HEADING_D * headingDerivative +
        CENTER_D  * centerDerivative;

    _app.Events->Publish(MotorCommand {
        NORMAL_SPEED * speed * speed,
        std::clamp(angular, -TURN_ANGULAR, TURN_ANGULAR)
    });
}

void MazeEngine::CalculateWalls()
{
    const auto leftHits = RayArc(FOV, TurnDirection::LEFT, RAY_DISTANCE);
    const auto rightHits = RayArc(FOV, TurnDirection::RIGHT, RAY_DISTANCE);

    const auto leftWall = FilterHitPoints(leftHits);
    const auto rightWall = FilterHitPoints(rightHits);

    if (leftWall.has_value() && rightWall.has_value() && std::abs(Vector2::Dot(leftWall.value().Direction, rightWall.value().Direction)) > 0.8) {
        _leftWall = leftWall;
        _rightWall = rightWall;

        _frontWall = std::nullopt;
        return;
    }

    const auto forwardHits = RayArc(FORWARD_FOV, TurnDirection::FORWARD, FORWARD_RAY_DISTANCE);
    const auto frontWall = FilterHitPoints(forwardHits);

    if (!frontWall.has_value())
        return;

    _frontWall = frontWall;

    _leftWall = std::nullopt;
    _rightWall = std::nullopt;
}

void MazeEngine::StartDecision(const bool left, const bool forward, const bool right)
{
    const auto now = std::chrono::steady_clock::now();
    if (now - _lastDecision < 5s)
        return;
    _lastDecision = now;

    const auto pose = _mapping->CurrentPose();

    switch (ChooseDirection(left, forward, right))
    {
        case TurnDirection::LEFT:
            _targetRotation = pose.rotation + M_PI_2;
            std::cout << "Turning left" << std::endl;
            break;

        case TurnDirection::RIGHT:
            _targetRotation = pose.rotation - M_PI_2;
                std::cout << "Turning right" << std::endl;
            break;

        case TurnDirection::FORWARD:
            _targetRotation = pose.rotation;
            std::cout << "Going forward" << std::endl;
            _state = NavState::FOLLOW_CORRIDOR;
            return;
    }

    _state = NavState::TURNING;
    _targetRotation = NormalizeAngle(_targetRotation);
}

void MazeEngine::ExecuteTurn()
{
    auto pose = _mapping->CurrentPose();

    float error = NormalizeAngle(_targetRotation - pose.rotation);

    if (std::abs(error) < 0.08f)
    {
        _state = NavState::FOLLOW_CORRIDOR;
        _prevTurnError = 0;
        return;
    }

    float derivative = NormalizeAngle(error - _prevTurnError) / _dt;
    _prevTurnError = error;

    float angular = error * TURN_P + derivative * TURN_D;
    angular = std::clamp(angular, -TURN_ANGULAR, TURN_ANGULAR);

    _app.Events->Publish(MotorCommand {
        TURN_SPEED,
        angular
    });
}

float MazeEngine::GetWallDistance(const TurnDirection side) const
{
    const auto pose = _mapping->CurrentPose();

    float offset = 0;

    switch (side) {
        case TurnDirection::FORWARD:
            offset = M_PI_2;
            break;
        case TurnDirection::LEFT:
            offset = M_PI;
            break;
        case TurnDirection::RIGHT:
            offset = 0;
            break;
    }

    RayHit rayHit;
    const auto hit = _mapping->RayCast(pose.position, Vector2::FromAngle(pose.rotation + offset), rayHit, 5.0);

    if (!hit) return 5.0;

    return Vector2::Distance(pose.position, rayHit.hit);
}

std::vector<RayHit> MazeEngine::RayArc(const float fov, const TurnDirection side, const float dst) const
{
    const auto pose = _mapping->CurrentPose();
    float baseAngle = 0;
    switch (side) {
        case TurnDirection::FORWARD:
            baseAngle = pose.rotation + M_PI_2;
            break;
        case TurnDirection::LEFT:
            baseAngle = pose.rotation + M_PI;
            break;
        case TurnDirection::RIGHT:
            baseAngle = pose.rotation;
            break;
    }

    const auto rad = fov * M_PI / 180.0;
    const auto angleStep = rad / (RAY_COUNT - 1);

    auto angle = baseAngle - rad * 0.5;

    std::vector<RayHit> hits;

    for (int i = 0; i <= RAY_COUNT; ++i)
    {
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

    for (size_t i = 1; i < hits.size(); ++i)
    {
        const auto prev = hits[i - 1];
        const auto curr = hits[i];

        const auto dot = Vector2::Dot(prev.normal, curr.normal);

        if (dot < 0.8f)
        {
            if (points.size() >= MIN_POINTS_PER_SEGMENT)
            {
                auto line = PcaFitter::FitLine(points);
                line.PointCount = static_cast<int>(points.size());

                lines.push_back(line);
            }

            points.clear();
        }

        points.push_back(curr.hit);
    }

    // Add final segment
    if (points.size() >= MIN_POINTS_PER_SEGMENT)
    {
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
    const Vector2 forward = pose.forward;

    std::optional<PcaFitter::FittedLine> bestLine;
    float bestScore = -std::numeric_limits<float>::infinity();

    for (const auto& line : lines)
    {
        // Wall direction alignment
        const float alignment =std::abs(Vector2::Dot(line.Direction.Normalized(), forward));

        // Distance from robot to line
        Vector2 toLine(
            line.Point.x - robotPos.x,
            line.Point.y - robotPos.y);

        Vector2 lineNormal = line.Normal;

        const float distToLine = std::abs(Vector2::Dot(toLine, lineNormal));

        // Reject very far walls
        if (distToLine > 3.0f)
            continue;

        // Reject badly aligned walls
        if (alignment < 0.5f)
            continue;

        // ---------- SCORE ----------
        float score = 0.0f;

        // Prefer parallel walls
        score += alignment * 5.0f;

        // Prefer closer walls
        score -= distToLine * 2.0f;

        // Prefer larger segments
        score += static_cast<float>(line.PointCount) * 0.5f;

        if (score > bestScore)
        {
            bestScore = score;
            bestLine = line;
        }
    }

    return bestLine;
}

TurnDirection MazeEngine::ChooseDirection(const bool left, const bool forward, const bool right)
{
    if (!_treasureCode.has_value() && !_exitCode.has_value()) {
        if (left) return TurnDirection::LEFT;
        if (forward) return TurnDirection::FORWARD;
        if (right) return TurnDirection::RIGHT;

        return TurnDirection::LEFT; // todo TurnDirection::BACK
    }

    auto aruCode = _treasureCode.has_value() ? _treasureCode.value() : _exitCode.value();

    _treasureCode = std::nullopt;
    _exitCode = std::nullopt;

    switch (aruCode.id % 10)
    {
        case 0: return TurnDirection::FORWARD;
        case 1: return TurnDirection::LEFT;
        case 2: return TurnDirection::RIGHT;

        default: return TurnDirection::LEFT;
    }
}

void MazeEngine::OnAruCode(CodeDetectedEvent aruCode)
{
    std::lock_guard lock(_mutex);

    if (aruCode.id >= 10)
        _treasureCode = aruCode;
    else
        _exitCode = aruCode;

    std::cout << "Code detected: " << aruCode.id << std::endl;
}

void MazeEngine::PublishWalls() const
{
    visualization_msgs::msg::MarkerArray markerArray;

    // Create delete markers with offset IDs (0-2 for deletes, 10-12 for adds)
    auto createDeleteMarker = [](int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "maze_walls";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        return marker;
    };

    // Add delete commands with IDs 10, 11, 12
    markerArray.markers.push_back(createDeleteMarker(10)); // left wall
    markerArray.markers.push_back(createDeleteMarker(11)); // right wall
    markerArray.markers.push_back(createDeleteMarker(12)); // front wall

    auto createMarker = [](const PcaFitter::FittedLine& line, int id, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "maze_walls";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1f;  // arrow shaft diameter
        marker.scale.y = 0.15f; // arrow head width
        marker.scale.z = 0.15f; // arrow head height
        marker.color.a = 1.0f;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

        // Start point
        geometry_msgs::msg::Point p1;
        p1.x = line.Point.x - line.Direction.x * 2.0f;
        p1.y = line.Point.y - line.Direction.y * 2.0f;

        // End point (direction)
        geometry_msgs::msg::Point p2;
        p2.x = line.Point.x + line.Direction.x * 2.0f;
        p2.y = line.Point.y + line.Direction.y * 2.0f;

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        return marker;
    };

    if (_leftWall.has_value())
    {
        markerArray.markers.push_back(createMarker(_leftWall.value(), 0, 1.0f, 0.0f, 0.0f)); // Red
    }

    if (_rightWall.has_value())
    {
        markerArray.markers.push_back(createMarker(_rightWall.value(), 1, 0.0f, 1.0f, 0.0f)); // Green
    }

    if (_frontWall.has_value())
    {
        markerArray.markers.push_back(createMarker(_frontWall.value(), 2, 0.0f, 0.0f, 1.0f)); // Blue
    }

    _publisher->publish(markerArray);
}

} // namespace Manhattan::Core