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
constexpr float SPLIT_THRESHOLD = 0.5f;
constexpr int MIN_POINTS_PER_SEGMENT = 4;

// Motor settings
constexpr float NORMAL_SPEED = 0.06f;
constexpr float TURN_SPEED = 0.0f;
constexpr float TURN_ANGULAR = 0.5f;

// PID tuning constants
constexpr float HEADING_P = 1.0f;
constexpr float CENTER_P = 0.5f;
constexpr float HEADING_D = 0.0f;
constexpr float CENTER_D = 0.0f;

constexpr float TURN_P = 0.4f;
constexpr float TURN_D = 0.01f;

static float NormalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.f * M_PI;
    while (angle < -M_PI) angle += 2.f * M_PI;

    return angle;
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
    _initialTimer = create_wall_timer(1s, [this] {
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

    const auto leftHits = RayArc(FOV, TurnDirection::LEFT, RAY_DISTANCE);
    const auto rightHits = RayArc(FOV, TurnDirection::RIGHT, RAY_DISTANCE);
    const auto forwardHits = RayArc(FORWARD_FOV, TurnDirection::FORWARD, FORWARD_RAY_DISTANCE);

    const auto leftWall = FilterHitPoints(leftHits);
    if (leftWall.has_value())
        _leftWall = leftWall;

    const auto rightWall = FilterHitPoints(rightHits);
    if (rightWall.has_value())
        _rightWall = rightWall;

    const auto frontWall = FilterHitPoints(forwardHits);
    if (frontWall.has_value())
        _frontWall = frontWall;

    float centerError = 0.0f;
    float headingError = 0.0f;

    float centerDerivative = 0.0f;
    float headingDerivative = 0.0f;

    if (_leftWall.has_value() && _rightWall.has_value() && abs(Vector2::Dot(_leftWall->Direction, _rightWall->Direction)) < 0.6){

        const Vector2 corridorDir = (_leftWall->Direction + _rightWall->Direction).Normalized();

        const Vector2 corridorNormal =
            Vector2::Perpendicular(corridorDir);

        const auto pose = _mapping->CurrentPose();

        const Vector2 centerPoint = (_leftWall->Point + _rightWall->Point) * 0.5f;

        const Vector2 errorVec = centerPoint - pose.position;

        centerError = Vector2::Dot(errorVec, corridorNormal);
        headingError = Vector2::Cross(corridorDir, pose.forward);

        centerDerivative = centerError - _prevCenterError;
        headingDerivative = headingError - _prevHeadingError;

        _prevCenterError = centerError;
        _prevHeadingError = headingError;

    } else if (_frontWall.has_value()) {
        const Vector2 robotFwd = _mapping->CurrentPose().forward;

        headingError = Vector2::Cross(_frontWall->Normal, robotFwd);

        headingDerivative = headingError - _prevHeadingError;
        _prevHeadingError = headingError;

        centerError = 0.0f;
        centerDerivative = 0.0f;
    }

    const float speed = std::clamp(frontDist / OPEN_THRESHOLD, 0.0f, 1.0f);
    const float angular =
        HEADING_P * headingError +
        CENTER_P  * centerError +
        HEADING_D * headingDerivative +
        CENTER_D  * centerDerivative;

    _app.Events->Publish(MotorCommand {
        NORMAL_SPEED * speed * speed,
        angular
    });
}

void MazeEngine::StartDecision(const bool left, const bool forward, const bool right)
{
    const auto now = std::chrono::steady_clock::now();
    if (now - _lastDecision < 2s)
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

    float rawError = _targetRotation - pose.rotation;
    float error = NormalizeAngle(rawError);

    if (std::abs(error) < 0.08f)
    {
        _state = NavState::FOLLOW_CORRIDOR;
        _prevTurnError = 0;
        return;
    }

    float rawDerivative = rawError - _prevTurnError;
    _prevTurnError = rawError;

    float derivative = NormalizeAngle(rawDerivative);

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

    auto createMarker = [](const PcaFitter::FittedLine& line, int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "maze_walls";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05f;
        marker.color.a = 1.0f;
        marker.color.r = 1.0f;

        geometry_msgs::msg::Point p1;
        p1.x = line.Point.x + line.Direction.x * 5.0f;
        p1.y = line.Point.y + line.Direction.y * 5.0f;

        geometry_msgs::msg::Point p2;
        p2.x = line.Point.x - line.Direction.x * 5.0f;
        p2.y = line.Point.y - line.Direction.y * 5.0f;

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        return marker;
    };

    if (_leftWall.has_value())
    {
        markerArray.markers.push_back(createMarker(_leftWall.value(), 0));
    }

    if (_rightWall.has_value())
    {
        markerArray.markers.push_back(createMarker(_rightWall.value(), 1));
    }

    if (_frontWall.has_value())
    {
        markerArray.markers.push_back(createMarker(_frontWall.value(), 2));
    }

    _publisher->publish(markerArray);
}

} // namespace Manhattan::Core