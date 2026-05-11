
#include "Components/ImuDriver.hpp"

#include "Components/OdometryEngine.hpp"

#include <nav_msgs/msg/detail/grid_cells__builder.hpp>

using namespace std;

constexpr auto baseImuTopic = "/bpc_prp_robot/imu";
constexpr auto imuTopic = "imu";

constexpr auto imuOrientationCovariance = 0.01;
constexpr auto imuAngularCovariance = 0.01;
constexpr auto imuLinearCovariance = 0.01;

namespace Manhattan::core {

ImuDriver::ImuDriver(const App& app)
    : RosDeviceDriver(app, "imu")
{
}

void ImuDriver::OnEnable()
{
    _imuSubscriber = create_subscription<sensor_msgs::msg::Imu>(
        baseImuTopic, 2,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->OnImu(msg); });

    _imuPublisher = create_publisher<sensor_msgs::msg::Imu>(imuTopic, 2);
}

void ImuDriver::OnDisable()
{
    _imuSubscriber.reset();
    _imuPublisher.reset();
}

void ImuDriver::OnImu(const sensor_msgs::msg::Imu::SharedPtr& msg) const
{
    auto resultMsg = *msg;

    resultMsg.header.frame_id = "base_link";

    resultMsg.angular_velocity_covariance = {
        imuAngularCovariance, 0.0, 0.0,
        0.0, imuAngularCovariance, 0.0,
        0.0, 0.0, imuAngularCovariance
    };

    resultMsg.linear_acceleration_covariance = {
        imuAngularCovariance, 0.0, 0.0,
        0.0, imuAngularCovariance, 0.0,
        0.0, 0.0, imuAngularCovariance
    };

    _imuPublisher->publish(resultMsg);
}

}
