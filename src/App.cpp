#include "App.h"

using namespace std;
using namespace rclcpp;

namespace Manhattan::Core
{
    App::App() {
        _executor = make_shared<executors::MultiThreadedExecutor>();
    }

    void App::Run() const
    {
        _executor->spin();
    }
}