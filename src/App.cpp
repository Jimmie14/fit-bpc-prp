#include "App.h"

using namespace std;
using namespace rclcpp;

namespace Manhattan::Core
{
    App::App() {
        _executor = make_shared<executors::MultiThreadedExecutor>();
        _tcpServer = make_unique<TcpServer>(12345);
    }

    void App::Run() const
    {
        _tcpServer->Start();
        _executor->spin();
    }
}
