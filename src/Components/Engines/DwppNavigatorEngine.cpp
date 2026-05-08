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
    , _config(_app.getConfig<config::DwppConfig>("navigation.dwpp"))
    , _kinematics({ })
    , _odometry({ })
{
    const auto geometry = _app.getConfig<config::DifferentialDriveGeometry>("geometry");

    _kinematics = DifferentialDriveKinematics(geometry);
    _odometry = DifferentialDriveOdometry(geometry);

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

        _path.Initialize(points);
    });

    app.events->Subscribe<messages::RobotPoseEvent>([this](const auto& event) {
        std::lock_guard guard(_lock);

        _pose = event.pose;
        _twist = event.twist;
    });

    _debugPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("nav/debug", 1);
}

void DwppNavigatorEngine::OnEnable()
{
    _updateTimer = create_wall_timer(duration<double>(_config.deltaTime), std::bind(&DwppNavigatorEngine::Update, this));

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

    _lookaheadPoint = _path.GetPointAtDistance(closest.distanceAlongPath + _config.lookaheadDistance);

    const auto vHalfRange = _config.linearAcceleration * _config.deltaTime;
    const auto vMin = clamp(_twist.linear - vHalfRange, -_config.maxLinearSpeed, _config.maxLinearSpeed);
    const auto vMax = clamp(_twist.linear + vHalfRange, -_config.maxLinearSpeed, _config.maxLinearSpeed);

    const auto wHalfRange = _config.angularAcceleration * _config.deltaTime;
    const auto wMin = clamp(_twist.angular - wHalfRange, -_config.maxAngularSpeed, _config.maxAngularSpeed);
    const auto wMax = clamp(_twist.angular + wHalfRange, -_config.maxAngularSpeed, _config.maxAngularSpeed);

    auto bestTwist = Twist::zero();
    auto bestScore = numeric_limits<double>::max();

    std::cout << "Simulating twist range: linear [" << vMin << ", " << vMax << "], angular [" << wMin << ", " << wMax << "]" << std::endl;

    _debugSimulations.clear();

    const double dv = (vMax - vMin) / std::max(1, _config.linearVelocitySamples - 1);
    const double dw = (wMax - wMin) / std::max(1, _config.angularVelocitySamples - 1);

    for (int i = 0; i < _config.linearVelocitySamples; i++) {
        const auto v = vMin + i * dv;

        for (int j = 0; j < _config.angularVelocitySamples; j++) {
            const auto w = wMin + j * dw;

            const auto twist = Twist(v, w);
            const auto score = Evaluate(twist);

            if (score > bestScore) continue;

            bestTwist = twist;
            bestScore = score;
        }
    }

    std::cout << "twist: " << bestTwist.toString() << std::endl;
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

    for (int i = 0; i < _config.simulationSteps; i++) {
        pose = _odometry.integrate(pose, step);

        const auto closest = _path.FindClosestPoint(pose.position);

        pathError += Vector2::distance(closest.position, pose.position);
    }

    pathError /= _config.simulationSteps;

    const auto d =  Vector2::distance(_lookaheadPoint, pose.position);

    _debugSimulations.push_back({ pose.position.toTf2(), pathError });

    return d;
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

    if (!_debugSimulations.empty())
    {
        double min_error = std::numeric_limits<double>::max();
        double max_error = std::numeric_limits<double>::lowest();

        for (const auto& [point, error] : _debugSimulations)
        {
            min_error = std::min(min_error, error);
            max_error = std::max(max_error, error);
        }

        const double range = std::max(1e-6, max_error - min_error);

        for (const auto& [point, error] : _debugSimulations)
        {
            const double t = (error - min_error) / range;

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
