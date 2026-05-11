#include "App.hpp"

#include <filesystem>
#include <toml++/toml.hpp>

using namespace std;
using namespace rclcpp;

namespace Manhattan::core {

App::App()
    : events(make_unique<EventBus>())
    , config(make_shared<ConfigManager>())
    , _executor(make_shared<executors::MultiThreadedExecutor>(ExecutorOptions(), 16))
{
    _executor->add_node(config);


    _tcpServer = make_shared<TcpServer>(12345);
}

void App::run() const
{
    _tcpServer->Start();
    _executor->spin();
}
} // namespace Manhattan::Core
