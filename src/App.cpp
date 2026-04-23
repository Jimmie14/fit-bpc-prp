#include "App.hpp"

using namespace std;
using namespace rclcpp;

namespace Manhattan::Core {
App::App()
    : Events(make_unique<EventBus>())
    , _executor(make_shared<executors::MultiThreadedExecutor>(ExecutorOptions(), 16))
{
    _tcpServer = make_shared<TcpServer>(12345);
}

void App::Run() const
{
    _tcpServer->Start();
    _executor->spin();
}
} // namespace Manhattan::Core
