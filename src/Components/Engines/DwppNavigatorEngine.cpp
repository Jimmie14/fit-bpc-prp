#include "DwppNavigatorEngine.hpp"

#include "App.hpp"
#include "Messages/Nav.hpp"
#include "Messages/RobotMode.hpp"
#include "MotorDriver.hpp"
#include "OdometryEngine.hpp"
#include "Viz/Marker.hpp"

constexpr auto deltaTime = 50ms;
constexpr auto lookaheadDistance = 0.2;

constexpr auto linearAcceleration = 0.03;
constexpr auto angularAcceleration = 0.01;

constexpr auto maxLinearSpeed = 0.3;
constexpr auto maxAngularSpeed = 0.3;

constexpr auto linearVelocitySamplingStep = 0.01;
constexpr auto angularVelocitySamplingStep = 0.01;

constexpr auto simulationSteps = 10;
constexpr auto simulationDeltaTime = 0.08;

namespace Manhattan::Core {

DwppNavigatorEngine::DwppNavigatorEngine(const App& app)
    : RosEngine(app, "navigator")
    , _kinematics(app.GetComponent<OdometryEngine>()->GetKinematics())
{
    _app.Events->Subscribe<Messages::RobotEnvironmentChangeEvent>([this](const auto& _) {
        std::lock_guard guard(_lock);

        _path = LinearPath();
    });

    _app.Events->Subscribe<Messages::RobotFollowPathEvent>([this](const auto& event) {

        vector<Vector2> points;

        for (const auto p : event.path) {
            points.push_back(Vector2(p));
        }

        std::lock_guard guard(_lock);

        _path.Initialize(points);
    });

    app.Events->Subscribe<Messages::RobotPoseEvent>([this](const auto& event) {
        std::lock_guard guard(_lock);

        _pose = event.pose;
        _twist = event.twist;
    });

    _debugPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("nav/debug", 1);
}

void DwppNavigatorEngine::OnEnable()
{
    _updateTimer = create_wall_timer(deltaTime, std::bind(&DwppNavigatorEngine::Update, this));

    _debugTimer = create_wall_timer(100ms, std::bind(&DwppNavigatorEngine::PublishDebug, this));
}

void DwppNavigatorEngine::OnDisable()
{
    _updateTimer.reset();
}

void DwppNavigatorEngine::Update()
{
    std::lock_guard guard(_lock);

    if (!_path.HasPath()) return;

    const auto closest = _path.FindClosestPoint(_pose.position);

    _lookaheadPoint = _path.GetPointAtDistance(closest.distanceAlongPath + lookaheadDistance);

    const auto vHalfRange = linearAcceleration * deltaTime.count();
    const auto vMin = clamp(_twist.linear - vHalfRange, -maxLinearSpeed, maxLinearSpeed);
    const auto vMax = clamp(_twist.linear + vHalfRange, -maxLinearSpeed, maxLinearSpeed);

    const auto wHalfRange = angularAcceleration * deltaTime.count();
    const auto wMin = clamp(_twist.angular - wHalfRange, -maxAngularSpeed, maxAngularSpeed);
    const auto wMax = clamp(_twist.angular + wHalfRange, -maxAngularSpeed, maxAngularSpeed);

    auto bestTwist = Twist::zero();
    auto bestScore = numeric_limits<double>::max();

    for (auto v = vMin; v <= vMax; v += linearVelocitySamplingStep) {
        for (auto w = wMin; w <= wMax; w += angularVelocitySamplingStep) {
            const auto twist = Twist(v, w);
            const auto score = Evaluate(twist);

            if (score > bestScore) continue;

            bestTwist = twist;
            bestScore = score;
        }
    }

    std::cout << "twist: " << bestTwist.toString() << std::endl;
    _app.Events->Publish(MotorCommand {
        .linear = bestTwist.linear,
        .angular = -bestTwist.angular
    });
}

double DwppNavigatorEngine::Evaluate(const Twist& twist) const
{
    auto [left, right] = _kinematics.inverse(twist);

    left *= simulationDeltaTime;
    right *= simulationDeltaTime;

    auto pose = _pose;

    auto pathError = 0.0;

    for (int i = 0; i < simulationSteps; i++) {
        pose = _kinematics.integrate(pose, left, right);

        const auto closest = _path.FindClosestPoint(pose.position);

        pathError += Vector2::distance(closest.position, pose.position);
    }

    pathError /= simulationSteps;

    return pathError;
}

void DwppNavigatorEngine::PublishDebug()
{
    std::lock_guard guard(_lock);

    auto builder = viz::marker::MarkerArrayBuilder();
    visualization_msgs::msg::Marker marker;

    builder.add(viz::marker::clear("map"));

    marker = viz::marker::point(_lookaheadPoint.toTf2(), "map");
    marker.color = viz::marker::color(0, 0, 1);

    builder.add(marker);

    marker = viz::marker::path(_path.getWaypoints(), "map");
    builder.add(marker);

    builder.add(viz::marker::twist(_pose, _twist, "map"));

    _debugPublisher->publish(builder.array);
}

}
