#include "LidarDriver.hpp"

#include "App.hpp"
#include "Vector2.hpp"

namespace Manhattan::Core {
constexpr auto LIDAR_TOPIC = "/bpc_prp_robot/lidar";

static double Median(double a, double b, double c)
{
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

static bool IsInRange(const double hit, const double minRange, const double maxRange)
{
    return hit > minRange && hit < maxRange;
}

LidarDriver::LidarDriver(const App& app)
    : RosDeviceDriver(app)
{
    _lidar_subscriber = _node->create_subscription<sensor_msgs::msg::LaserScan>(
        LIDAR_TOPIC, 1, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { LidarFilter(msg); });
}

void LidarDriver::LidarFilter(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
    const auto length = msg->ranges.size();
    const auto angleStep = M_PI * 2.0 / static_cast<double>(length);
    auto angle = M_PI * 0.5; // forward dir
    auto pointIndex = 0;

    _points.resize(length);

    if (IsInRange(msg->ranges[0], msg->range_min, msg->range_max))
        _points[pointIndex++] = Vector2(0, msg->ranges[0]);

    for (auto i = 1; i < length - 1; ++i) {
        angle += angleStep;

        const float hit = msg->ranges[i];
        if (!IsInRange(hit, msg->range_min, msg->range_max))
            continue;

        const auto median = Median(msg->ranges[i - 1], hit, msg->ranges[i + 1]);
        _points[pointIndex++] = Vector2(std::cos(angle), std::sin(angle)) * median;
    }

    angle += angleStep;
    if (IsInRange(msg->ranges[length - 1], msg->range_min, msg->range_max))
        _points[pointIndex++] = Vector2(msg->ranges[length - 1] * std::cos(angle), msg->ranges[length - 1] * std::sin(angle));

    _points.resize(pointIndex);

    _app.Events.Publish(LidarScan { _points });
}
} // namespace Manhattan::Core
