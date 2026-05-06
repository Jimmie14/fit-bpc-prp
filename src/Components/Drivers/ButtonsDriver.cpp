
#include "ButtonsDriver.hpp"

#include "App.hpp"
#include "MappingEngine.hpp"
#include "OdometryEngine.hpp"
#include <nav_msgs/msg/detail/grid_cells__builder.hpp>

constexpr auto buttonsTopic = "/bpc_prp_robot/buttons";


namespace Manhattan::Core {

ButtonsDriver::ButtonsDriver(const App& app)
    : RosDeviceDriver(app, "buttons")
{
}

void ButtonsDriver::OnEnable()
{
    _subscriber = create_subscription<std_msgs::msg::UInt8>(buttonsTopic, 2, [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        this->OnButtons(*msg);
    });
}

void ButtonsDriver::OnDisable()
{
    _subscriber.reset();
}

void ButtonsDriver::OnButtons(const std_msgs::msg::UInt8& msg)
{
    const auto index = static_cast<int>(msg.data);
    if (index == 0) {
        _app.Events->Publish(RobotResetEvent { });
    }

    if (index == 1) {
        const auto oldMode = _mode;
        _mode.reverse = !_mode.reverse;

        _app.Events->Publish(RobotModeChangeEvent { .oldMode = oldMode, .newMode = _mode});
    }
}

}
