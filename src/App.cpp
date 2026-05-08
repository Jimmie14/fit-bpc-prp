#include "App.hpp"

#include <filesystem>
#include <toml++/toml.hpp>

using namespace std;
using namespace rclcpp;

namespace Manhattan::core {

App::App()
    : events(make_unique<EventBus>())
    , _executor(make_shared<executors::MultiThreadedExecutor>(ExecutorOptions(), 16))
{
    _tcpServer = make_shared<TcpServer>(12345);

    std::cout << std::filesystem::current_path() << std::endl;

    const auto node = std::make_shared<Node>("manhattan_config");

    const auto configPath = node->declare_parameter("config", "manhattan.toml");

    _config = config::Config::load(configPath);
}

void App::run() const
{
    _tcpServer->Start();
    _executor->spin();
}
} // namespace Manhattan::Core
