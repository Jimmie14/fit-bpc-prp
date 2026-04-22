#include "App.hpp"

using namespace std;
using namespace rclcpp;

namespace Manhattan::Core {
App::App()
{
    _executor = make_shared<executors::MultiThreadedExecutor>(ExecutorOptions(), 16);
    _tcpServer = make_shared<TcpServer>(12345);

    _node = make_shared<Node>("manhattan");
    _executor->add_node(_node);
}

void App::Run() const
{
    _tcpServer->Start();
    _executor->spin();
}
} // namespace Manhattan::Core
