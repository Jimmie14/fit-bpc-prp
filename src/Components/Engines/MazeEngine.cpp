#include "MazeEngine.hpp"

#include "Viz/Grid.hpp"

using namespace std;

namespace Manhattan::Core {

using namespace Manhattan::nav;

constexpr float OPEN_THRESHOLD = 0.30f;
constexpr float WALL_TARGET = 0.18f;
constexpr float RAY_DISTANCE = 1.0f;

constexpr float NORMAL_SPEED = 0.05f;
constexpr float TURN_SPEED = 0.0f;
constexpr float TURN_ANGULAR = 1.5f;

// PID tuning constants
constexpr float FOLLOW_P = 0.6f;
constexpr float FOLLOW_D = 0.015f;

constexpr float TURN_P = 0.5f;
constexpr float TURN_D = 0.015f;
constexpr float RECENTER_P = 2.0f;

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
    _app.Events->Subscribe<CodeDetectedEvent>([this](const CodeDetectedEvent& event) {
        OnAruCode(event);
    });
}

void MazeEngine::OnEnable() {
    _initialTimer = create_wall_timer(1s, [this] {
        _timer = create_wall_timer(10ms, [this] { Update(); });
        _initialTimer.reset();
    });
}

void MazeEngine::OnDisable() {
    _timer.reset();
    _initialTimer.reset();
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

            case NavState::RECENTER:
                Recenter();
                break;

            default:
                break;
    }
}

void MazeEngine::FollowCorridor()
{
    float leftDist = GetLeftWallDistance();
    float rightDist = GetRightWallDistance();
    float frontDist = GetFrontWallDistance();
    float behindDist = GetBehindWallDistance();

    bool openLeft = leftDist > OPEN_THRESHOLD;
    bool openRight = rightDist > OPEN_THRESHOLD;
    bool openFront = frontDist > OPEN_THRESHOLD;
    bool openBehind = behindDist > OPEN_THRESHOLD;

    const bool tJunction = openLeft && openRight && !openFront;
    const bool cornerLeft = openLeft && !openRight && !openFront;
    const bool cornerRight = openRight && !openLeft && !openLeft;
    const bool xJunction = openLeft && openRight && openFront;
    const bool deadEnd = !openLeft && !openRight && !openFront;

    if (tJunction || cornerLeft || cornerRight || xJunction || deadEnd)
    {
        StartDecision(openLeft, openFront, openRight);
        return;
    }

    float error = leftDist - rightDist;
    float derivative = error - _prevError;
    _prevError = error;

    float speed = frontDist < OPEN_THRESHOLD * 0.7f ? 0 : 1;
    float angular = error * FOLLOW_P + derivative * FOLLOW_D;

    _app.Events->Publish(MotorCommand {
        NORMAL_SPEED * speed,
        angular
    });
}

void MazeEngine::StartDecision(bool left, bool forward, bool right)
{
    auto now = std::chrono::steady_clock::now();
    if (now - _lastDecision < 2s)
        return;
    _lastDecision = now;

    auto pose = _mapping->CurrentPose();
    auto dir = ChooseDirection(left, forward, right);

    switch (dir)
    {
        case TurnDirection::LEFT:
            _targetRotation = pose.rotation + M_PI_2;
            break;

        case TurnDirection::RIGHT:
            _targetRotation = pose.rotation;
            break;

        case TurnDirection::FORWARD:
            _targetRotation = pose.rotation + M_PI * 0.5f;
            break;

        case TurnDirection::BACK:
            _targetRotation = pose.rotation - M_PI * 0.5f;
            break;
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
        _state = NavState::RECENTER;
        return;
    }

    float derivative = error - _prevTurnError;
    _prevTurnError = error;

    float angular = error * TURN_P + derivative * TURN_D;
    angular = std::clamp(angular, -TURN_ANGULAR, TURN_ANGULAR);

    _app.Events->Publish(MotorCommand {
        TURN_SPEED,
        angular
    });
}

void MazeEngine::Recenter()
{
    float leftDist = GetLeftWallDistance();
    float rightDist = GetRightWallDistance();

    float error = leftDist - rightDist;

    if (std::abs(error) < 0.03f)
    {
        _state = NavState::FOLLOW_CORRIDOR;
        return;
    }

    float angular = error * RECENTER_P;

    _app.Events->Publish(MotorCommand {
        TURN_SPEED,
        angular
    });
}

float MazeEngine::GetLeftWallDistance()
{
    auto pose = _mapping->CurrentPose();

    RayHit rayHit;
    const auto hit = _mapping->RayCast(pose.position, Vector2::FromAngle(pose.rotation + M_PI), rayHit, RAY_DISTANCE);

    if (!hit) return RAY_DISTANCE;

    return Vector2::Distance(pose.position, rayHit.hit);
}

float MazeEngine::GetRightWallDistance()
{
    auto pose = _mapping->CurrentPose();

    RayHit rayHit;
    const auto hit = _mapping->RayCast(pose.position, Vector2::FromAngle(pose.rotation), rayHit, RAY_DISTANCE);

    if (!hit) return RAY_DISTANCE;

    return Vector2::Distance(pose.position, rayHit.hit);
}

float MazeEngine::GetFrontWallDistance()
{
    auto pose = _mapping->CurrentPose();

    RayHit rayHit;
    const auto hit = _mapping->RayCast(pose.position, Vector2::FromAngle(pose.rotation + M_PI / 2.0), rayHit, RAY_DISTANCE);

    if (!hit) return RAY_DISTANCE;

    return Vector2::Distance(pose.position, rayHit.hit);
}
float MazeEngine::GetBehindWallDistance()
{
    auto pose = _mapping->CurrentPose();

    RayHit rayHit;
    const auto hit = _mapping->RayCast(pose.position, Vector2::FromAngle(pose.rotation - M_PI / 2.0), rayHit, RAY_DISTANCE);

    if (!hit) return RAY_DISTANCE;

    return Vector2::Distance(pose.position, rayHit.hit);
}

TurnDirection MazeEngine::ChooseDirection(bool left, bool forward, bool right)
{
    switch (_currentDecision)
    {
        case TurnDirection::LEFT:
            if (left) return TurnDirection::LEFT;
            break;

        case TurnDirection::FORWARD:
            if (forward) return TurnDirection::FORWARD;
            break;

        case TurnDirection::RIGHT:
            if (right) return TurnDirection::RIGHT;
            break;
    }

    // fallback priority
    if (left) return TurnDirection::LEFT;
    if (forward) return TurnDirection::FORWARD;
    if (right) return TurnDirection::RIGHT;

    return TurnDirection::LEFT;
}

void MazeEngine::OnAruCode(CodeDetectedEvent aruCode)
{
    std::lock_guard lock(_mutex);

    if (aruCode.id >= 10)
        _treasureCode = aruCode;
    else
        _exitCode = aruCode;

    switch (aruCode.id % 10)
    {
        case 0:
            _currentDecision = TurnDirection::FORWARD;
            break;

        case 1:
            _currentDecision = TurnDirection::LEFT;
            break;

        case 2:
            _currentDecision = TurnDirection::RIGHT;
            break;
        default:
            break;
    }

    std::cout << "Code detected: " << aruCode.id << std::endl;
}

} // namespace Manhattan::Core