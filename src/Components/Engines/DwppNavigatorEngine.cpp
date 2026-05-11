#include "Components/DwppNavigatorEngine.hpp"

#include "App.hpp"
#include "Messages/Nav.hpp"
#include "Messages/RobotMode.hpp"
#include "Components/OdometryEngine.hpp"
#include "Viz/Marker.hpp"

namespace Manhattan::core {

using namespace Manhattan::messages;

DwppNavigatorEngine::DwppNavigatorEngine(const App& app)
    : RosEngine(app, "navigator")
    , _kinematics({ })
    , _odometry({ })
{
    _app.config->watch<DwppConfig>("navigation.dwpp", [this](const DwppConfig& config) {
        _config = config;
    });

    _app.config->watch<DifferentialDriveGeometry>("geometry", [this](const DifferentialDriveGeometry& geometry) {
        _kinematics = DifferentialDriveKinematics(geometry);
        _odometry = DifferentialDriveOdometry(geometry);
    });


    _app.events->Subscribe<RobotEnvironmentChangeEvent>([this](const auto& _) {
        std::lock_guard guard(_lock);

        _path = LinearPath();
    });

    _app.events->Subscribe<RobotFollowPathEvent>([this](const auto& event) {

        vector<Vector2> points;

        for (const auto p : event.path) {
            points.push_back(Vector2(p));
        }

        std::lock_guard guard(_lock);

        _path.init(points);
    });

    app.events->Subscribe<RobotPoseEvent>([this](const auto& event) {
        std::lock_guard guard(_lock);

        _pose = event.pose;
        _twist = event.twist;
    });

    _debugPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("nav/debug", 1);
}

void DwppNavigatorEngine::OnEnable()
{
    _updateTimer = create_wall_timer(duration<double>(_config.deltaTime), [this] {
        Update();
    });

    _debugTimer = create_wall_timer(100ms, [this] {
        PublishDebug();
    });
}

void DwppNavigatorEngine::OnDisable()
{
    _updateTimer.reset();
}

void DwppNavigatorEngine::Update()
{
    std::lock_guard guard(_lock);

    if (!_path.HasPath()) return;

    const auto closest = _path.findClosestPoint(_pose.position);

    _lookaheadPoint = _path.GetPointAtDistance(closest.distanceAlongPath + _config.lookaheadDistance);

    const auto vHalfRange = _config.linearAcceleration * _config.deltaTime;
    const auto vMin = clamp(_twist.linear - vHalfRange, -_config.maxLinearSpeed, _config.maxLinearSpeed);
    const auto vMax = clamp(_twist.linear + vHalfRange, -_config.maxLinearSpeed, _config.maxLinearSpeed);

    const auto wHalfRange = _config.angularAcceleration * _config.deltaTime;
    const auto wMin = clamp(_twist.angular - wHalfRange, -_config.maxAngularSpeed, _config.maxAngularSpeed);
    const auto wMax = clamp(_twist.angular + wHalfRange, -_config.maxAngularSpeed, _config.maxAngularSpeed);

    auto bestTwist = Twist::zero();
    auto bestScore = -16.0;

    _debugSimulations.clear();

    const double dv = (vMax - vMin) / std::max(1, _config.linearVelocitySamples - 1);
    const double dw = (wMax - wMin) / std::max(1, _config.angularVelocitySamples - 1);

    for (int i = 0; i < _config.linearVelocitySamples; i++) {
        const auto v = vMin + i * dv;

        for (int j = 0; j < _config.angularVelocitySamples; j++) {
            const auto w = wMin + j * dw;

            const auto twist = Twist(v, w);

            const auto score = Evaluate(twist);

            if (score < bestScore) continue;

            bestTwist = twist;
            bestScore = score;
        }
    }

    _app.events->Publish(MotorCommandEvent {
        .twist = bestTwist
    });
}

double DwppNavigatorEngine::Evaluate(const Twist& twist)
{
    auto angular = _kinematics.inverse(twist);

    angular.left *= _config.simulationDeltaTime;
    angular.right *= _config.simulationDeltaTime;

    const auto step = _kinematics.angularToLinear(angular);

    auto pose = _pose;

    auto pathError = 0.0;
    auto headingError = 0.0;
    auto progressReward = 0.0;

    auto previous = _path.findClosestPoint(pose.position);

    for (int i = 0; i < _config.simulationSteps; i++) {
        pose = _odometry.integrate(pose, step);

        const auto closest = _path.findClosestPoint(pose.position);

        pathError += Vector2::distance(closest.position, pose.position);

        const auto lookahead = _path.GetPointAtDistance(closest.distanceAlongPath + _config.lookaheadDistance);
        const auto toLookahead = (lookahead - pose.position).normalized();

        headingError += 1.0 - Vector2::dot(pose.forward(), toLookahead);

        progressReward += closest.distanceAlongPath - previous.distanceAlongPath;

        previous = closest;
    }

    auto const time = _config.simulationDeltaTime * _config.simulationSteps;

    pathError /= time;
    headingError /= time;
    progressReward /= time;

    const auto score =
        + _config.progressRewardWeight * progressReward
        - _config.pathErrorCostWeight * pathError
        - _config.headingErrorCostWeight * headingError;

    _debugSimulations.emplace_back(pose.position.toTf2(), score);
    return score;
}

void DwppNavigatorEngine::PublishDebug()
{
    std::lock_guard guard(_lock);

    auto builder = viz::marker::MarkerArrayBuilder();

    builder.add(viz::marker::clear("map"));

    auto marker = viz::marker::point(_lookaheadPoint.toTf2(), "map");
    marker.color = viz::marker::color(0, 0, 1);

    builder.add(marker);

    builder.add(viz::marker::path(_path.waypoints(), "map"));

    builder.add(viz::marker::twist(_pose, _twist, "map"));

    if (!_debugSimulations.empty())
    {
        double minScore = std::numeric_limits<double>::max();
        double maxScore = std::numeric_limits<double>::lowest();

        for (const auto& error : _debugSimulations | views::values)
        {
            minScore = std::min(minScore, error);
            maxScore = std::max(maxScore, error);
        }

        const double range = std::max(1e-6, maxScore - minScore);

        for (const auto& [point, score] : _debugSimulations)
        {
            const double t = (maxScore - score) / range;

            marker = viz::marker::point(point, "map");

            marker.scale.x = 0.06;
            marker.scale.y = 0.06;
            marker.scale.z = 0.06;

            marker.color = viz::marker::color(static_cast<float>(t), static_cast<float>(1.0 - t), 0.0f);

            builder.add(marker);
        }
    }

    _debugPublisher->publish(builder.array);
}

}
