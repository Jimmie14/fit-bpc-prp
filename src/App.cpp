#include "App.h"

using namespace std;
using namespace rclcpp;

namespace Manhattan::Core
{
    App::App()
    {
        _executor = make_shared<executors::MultiThreadedExecutor>();
        _tcpServer = make_unique<TcpServer>(12345, *this);

        _node = make_shared<Node>("manhattan");
        _executor->add_node(_node);
    }

    void App::Run() const
    {
        _tcpServer->Start();
        _executor->spin();
    }

    void App::ReceiveMessage(const std::string& message) const
    {
        std::istringstream iss(message);
        std::string command;
        std::string value;

        iss >> command;
        std::getline(iss, value);
        // Remove leading space from value if present
        if (!value.empty() && value[0] == ' ') value.erase(0, 1);

        if (command == "STOP"){
            if (_stopCallback) _stopCallback();
        }
        else if (command == "MOVE") {
            double linear = 0.0, angular = 0.0;
            std::istringstream vss(value);
            std::string token;
            while (vss >> token) {
                auto pos = token.find('=');
                if (pos != std::string::npos) {
                    std::string key = token.substr(0, pos);
                    std::string val = token.substr(pos + 1);
                    double dval = std::stod(val);
                    if (key == "linear") linear = dval;
                    else if (key == "angular") angular = dval;
                }
            }

            if (_moveCallback) {
                _moveCallback(linear, angular);
            }
        }
    }
}
