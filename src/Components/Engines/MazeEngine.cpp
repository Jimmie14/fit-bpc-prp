#include "MazeEngine.hpp"

#include "Nav/PcaFilter.hpp"
#include "Viz/Grid.hpp"

using namespace std;

namespace Manhattan::Core {

using namespace Manhattan::nav;

// Junction detection
constexpr float CORRIDOR_SIZE = 0.4f;

// RayArc settings
constexpr int RAY_COUNT = 9;
constexpr float RAY_DISTANCE = 5.0f;
constexpr float FOV = 20.0f;

// walls filtering
constexpr int MIN_POINTS_PER_SEGMENT = 2;

// Motor settings
constexpr float NORMAL_SPEED = 0.1f;
constexpr float TURN_SPEED = 0.0f;
constexpr float TURN_ANGULAR = 0.5f;

// PID tuning constants
constexpr float HEADING_P = 0.7f;
constexpr float HEADING_D = 0.05f;

constexpr float TURN_P = 0.5f;
constexpr float TURN_D = 0.01f;

constexpr float JUNCTION_CONFIRMATION_TIME = 0.5f;

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
    auto projection = Vector2::Dot(toTarget, normalizedDir);
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
        PublishHeading();
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
    // const float leftDist = GetWallDistance(TurnDirection::LEFT);
    // const float rightDist = GetWallDistance(TurnDirection::RIGHT);
    // const float frontDist = GetWallDistance(TurnDirection::FORWARD);

    const bool openLeft = !WallInDirection(TurnDirection::LEFT); // leftDist > OPEN_THRESHOLD;
    const bool openRight = !WallInDirection(TurnDirection::RIGHT); // rightDist > OPEN_THRESHOLD;
    const bool openFront = !WallInDirection(TurnDirection::FORWARD); // frontDist > OPEN_THRESHOLD;

    // std::cout << "Front: " << openFront << " Left: " << openLeft << " Right: " << openRight << std::endl;

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

    float headingError = 0.0f;
    float headingDerivative = 0.0f;

    CalculateWalls();

    const auto pose = _mapping->CurrentPose();

    if (_leftWall.has_value() && _rightWall.has_value()) {
        auto leftDir  = _leftWall->Direction.Normalized();
        auto rightDir = _rightWall->Direction.Normalized();

        if (Vector2::Dot(leftDir, rightDir) < 0.0f)
            rightDir = -rightDir;

        const Vector2 corridorDir = (leftDir + rightDir).Normalized();

        const auto leftPoint = ClosestPointOnLine(leftDir, _leftWall->Point, pose.position);
        const auto rightPoint = ClosestPointOnLine(rightDir, _rightWall->Point, pose.position);

        const auto center = (leftPoint + rightPoint) * 0.5f;

        _heading = corridorDir;
        _center = center;

        const auto target = center + corridorDir;

        headingError = static_cast<float>(Vector2::SignedAngle(pose.forward, (target - pose.position).Normalized()));
        headingDerivative = (headingError - _prevHeadingError) / _dt;
        _prevHeadingError = headingError;
    }

    auto frontDist = 0.0;
    for (auto hit : RayArc(5.0f, TurnDirection::FORWARD, RAY_DISTANCE)) {
        const auto dst = Vector2::Distance(hit.hit, pose.position);
        frontDist += dst;
    }
    frontDist /= RAY_COUNT;

    const float speed = std::clamp(static_cast<float>(frontDist) / CORRIDOR_SIZE, 0.0f, 1.0f);
    const float angular =
        HEADING_P * headingError +
        HEADING_D * headingDerivative;

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

    if (!leftWall.has_value() || !rightWall.has_value() || std::abs(Vector2::Dot(leftWall.value().Direction, rightWall.value().Direction)) < 0.9) return;

    _leftWall = leftWall;
    _rightWall = rightWall;

    const auto pose = _mapping->CurrentPose();
    const auto closestPointOnLeft = ClosestPointOnLine(leftWall->Direction, leftWall->Point, pose.position);
    const auto dstOnLeft = Vector2::Distance(closestPointOnLeft, pose.position);

    const auto closestPointOnRight = ClosestPointOnLine(rightWall->Direction, rightWall->Point, pose.position);
    const auto dstOnRight = Vector2::Distance(closestPointOnRight, pose.position);

    constexpr auto wallDistance = 0.4f;

    if (dstOnLeft > wallDistance)
        _leftWall->Point = (closestPointOnLeft - pose.position).Normalized() * (wallDistance - dstOnRight) + pose.position;

    if (dstOnRight > wallDistance)
        _rightWall->Point = (closestPointOnRight - pose.position).Normalized() * (wallDistance - dstOnLeft) + pose.position;

    if (Vector2::Dot(_leftWall->Direction, pose.forward) < 0.0f)
        _leftWall->Direction = -_leftWall->Direction;

    if (Vector2::Dot(_rightWall->Direction, pose.forward) < 0.0f)
        _rightWall->Direction = -_rightWall->Direction;
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
            _targetRotation = static_cast<float>(pose.rotation + M_PI_2);
            std::cout << "Turning left" << std::endl;
            break;

        case TurnDirection::RIGHT:
            _targetRotation = static_cast<float>(pose.rotation - M_PI_2);
                std::cout << "Turning right" << std::endl;
            break;

        case TurnDirection::FORWARD:
            _targetRotation = static_cast<float>(pose.rotation);
            std::cout << "Going forward" << std::endl;
            _state = NavState::FOLLOW_CORRIDOR;
            return;
    }

    _state = NavState::TURNING;
    _targetRotation = NormalizeAngle(_targetRotation);
}

void MazeEngine::ExecuteTurnState()
{
    const auto pose = _mapping->CurrentPose();
    const float error = NormalizeAngle(_targetRotation - static_cast<float>(pose.rotation));

    if (std::abs(error) < 0.08f)
    {
        _state = NavState::RECENTER;
        _prevTurnError = 0;

        CalculateWalls();
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

void MazeEngine::RecenterState()
{
    if (!_leftWall.has_value() || !_rightWall.has_value()) return;

    const auto pose = _mapping->CurrentPose();

    const auto leftDir = _leftWall->Direction.Normalized();
    const auto rightDir = _rightWall->Direction.Normalized();

    Vector2 corridorDir = (leftDir + rightDir).Normalized();

    const auto leftPoint = ClosestPointOnLine(leftDir, _leftWall->Point, pose.position);
    const auto rightPoint = ClosestPointOnLine(rightDir, _rightWall->Point, pose.position);

    const auto center = (leftPoint + rightPoint) * 0.5f;

    _heading = corridorDir;
    _center = center;

    const auto target = center + corridorDir;

    const auto headingError = static_cast<float>(Vector2::SignedAngle(pose.forward, (target - pose.position).Normalized()));

    if (std::abs(headingError) < 0.08f)
    {
        _state = NavState::FOLLOW_CORRIDOR;
        _prevTurnError = 0;
        return;
    }

    const auto headingDerivative = (headingError - _prevHeadingError) / _dt;
    _prevHeadingError = headingError;

    const float angular = TURN_P * headingError + TURN_D * headingDerivative;

    _app.Events->Publish(MotorCommand {
        0,
        std::clamp(angular, -TURN_ANGULAR, TURN_ANGULAR)
    });
}

bool MazeEngine::WallInDirection(const TurnDirection side) const
{
    const auto pose = _mapping->CurrentPose();
    const auto offset = (CORRIDOR_SIZE * 0.5f) * 0.5f; // half corridor size, then offset it to have center with robot
    const auto stepSize = offset / RAY_COUNT;

    auto direction = pose.forward;

    switch (side) {
    case TurnDirection::FORWARD:
        break;
    case TurnDirection::LEFT:
        direction = -Vector2::Perpendicular(direction);
        break;
    case TurnDirection::RIGHT:
        direction = Vector2::Perpendicular(direction);
        break;
    }

    auto perpendicularDir = Vector2::Perpendicular(direction);
    auto origin = pose.position + -perpendicularDir * offset;

    for (auto i = 0; i < RAY_COUNT; ++i) {
        RayHit rayHit;
        if (_mapping->RayCast(origin, direction, rayHit, CORRIDOR_SIZE))
            return true;

        origin = origin + perpendicularDir * stepSize;
    }

    return false;
}

std::vector<RayHit> MazeEngine::RayArc(const float fov, const TurnDirection side, const float dst) const
{
    const auto pose = _mapping->CurrentPose();
    float baseAngle = 0;
    switch (side) {
        case TurnDirection::FORWARD:
            baseAngle = static_cast<float>(pose.rotation + M_PI_2);
            break;
        case TurnDirection::LEFT:
            baseAngle = static_cast<float>(pose.rotation + M_PI);
            break;
        case TurnDirection::RIGHT:
            baseAngle = static_cast<float>(pose.rotation);
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
    auto bestScore = -std::numeric_limits<double>::infinity();

    for (const auto& line : lines)
    {
        const auto alignment = std::abs(Vector2::Dot(line.Direction.Normalized(), forward));

        Vector2 toLine(
            line.Point.x - robotPos.x,
            line.Point.y - robotPos.y);

        Vector2 lineNormal = line.Normal;

        const auto distToLine = std::abs(Vector2::Dot(toLine, lineNormal));

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

    // if (aruCode.id >= 10)
    //     _treasureCode = aruCode;
    // else
    //     _exitCode = aruCode;

    std::cout << "Code detected: " << aruCode.id << std::endl;
}

void MazeEngine::PublishWalls() const
{
    visualization_msgs::msg::MarkerArray markerArray;

    auto createDeleteMarker = [](int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "maze_walls";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        return marker;
    };

    // Delete old markers with IDs 0, 1, 2
    markerArray.markers.push_back(createDeleteMarker(0)); // left wall
    markerArray.markers.push_back(createDeleteMarker(1)); // right wall
    markerArray.markers.push_back(createDeleteMarker(2)); // front wall

    auto createMarker = [](const PcaFitter::FittedLine& line, int id, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "maze_walls";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1f;
        marker.scale.y = 0.15f;
        marker.scale.z = 0.15f;
        marker.color.a = 1.0f;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

        geometry_msgs::msg::Point p1;
        p1.x = line.Point.x - line.Direction.x * 2.0f;
        p1.y = line.Point.y - line.Direction.y * 2.0f;

        geometry_msgs::msg::Point p2;
        p2.x = line.Point.x + line.Direction.x * 2.0f;
        p2.y = line.Point.y + line.Direction.y * 2.0f;

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        return marker;
    };

    if (_leftWall.has_value())
        markerArray.markers.push_back(createMarker(_leftWall.value(), 3, 1.0f, 0.0f, 0.0f));

    if (_rightWall.has_value())
        markerArray.markers.push_back(createMarker(_rightWall.value(), 4, 0.0f, 1.0f, 0.0f));

    // if (_frontWall.has_value())
    //     markerArray.markers.push_back(createMarker(_frontWall.value(), 5, 0.0f, 0.0f, 1.0f));

    _publisher->publish(markerArray);
}

void MazeEngine::PublishHeading() const
{
    visualization_msgs::msg::MarkerArray markerArray;

    // Delete previous marker
    visualization_msgs::msg::Marker deleteMarker;
    deleteMarker.header.frame_id = "map";
    deleteMarker.ns = "maze_heading";
    deleteMarker.id = 20;  // Use different ID for delete
    deleteMarker.action = visualization_msgs::msg::Marker::DELETE;
    markerArray.markers.push_back(deleteMarker);

    // Create heading arrow marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "maze_heading";
    marker.id = 21;  // Use unique ID (different from walls 0-2)
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1f;  // arrow shaft diameter
    marker.scale.y = 0.15f; // arrow head width
    marker.scale.z = 0.15f; // arrow head height
    marker.color.a = 1.0f;
    marker.color.r = 1.0f;  // White
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;


    const auto pose = _mapping->CurrentPose();

    // Start point
    geometry_msgs::msg::Point p1;
    p1.x = pose.position.x; //_center.x;
    p1.y = pose.position.y; //_center.y;
    p1.z = 0.0f;

    const auto target = _center + _heading;

    // End point (direction)
    geometry_msgs::msg::Point p2;
    p2.x = pose.position.x + (target - pose.position).Normalized().x; //  _center.x + _heading.x;
    p2.y = pose.position.y + (target - pose.position).Normalized().y; // _center.y + _heading.y;
    p2.z = 0.0f;

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    markerArray.markers.push_back(marker);

    _publisher->publish(markerArray);
}

} // namespace Manhattan::Core