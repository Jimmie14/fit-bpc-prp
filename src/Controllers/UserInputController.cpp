#include "UserInputController.hpp"

#include "App.h"
#include "Kinematics.hpp"
using namespace std;
using namespace rclcpp;

namespace Manhattan::Core
{
    UserInputController::UserInputController(const App& app) : BaseController(app), _kinematics(app.GetController<RobotOdometry>()->GetKinematics())
    {
        _tcpServer = app.GetTcpServer();

        _motorController = app.GetController<MotorController>();
        _lineController = app.GetController<LineController>();

        Enable();
    }

    void UserInputController::ReceiveMessage(const std::vector<uint8_t>& data)
    {
        std::istringstream iss(std::string(data.begin(), data.end()));
        std::string command;
        std::string value;

        iss >> command;
        std::getline(iss, value);

        // Remove leading space from value if present
        if (!value.empty() && value[0] == ' ') value.erase(0, 1);

        DecodeMessage(command, value);
    }

    void UserInputController::DecodeMessage(const std::string& command, const std::string& value)
    {
        if (command == "STOP"){
            _motorController->SetForce(0, 0);
            if (_activeController) _activeController->Disable();

            _activeController.reset();
        }
        else if (command == "MOVE") {
            ParseMoveCommand(value);
        } else if (command == "MODE" && value == "LINE_FOLLOW") {
            _lineController->Enable();
            _activeController = std::dynamic_pointer_cast<BaseController>(_lineController);
        }
    }

    void UserInputController::ParseMoveCommand(const std::string& value) const
    {
        double linear = 0.0, angular = 0.0;
        std::istringstream vss(value);
        std::string token;

        while (vss >> token) {
            if (const auto pos = token.find('='); pos != std::string::npos) {
                std::string key = token.substr(0, pos);
                std::string val = token.substr(pos + 1);
                const double dVal = std::stod(val);
                if (key == "linear") linear = dVal;
                else if (key == "angular") angular = dVal;
            }
        }

        const auto robotSpeed = RobotSpeed(linear / 5, angular);
        auto [left, right] = _kinematics.inverse(robotSpeed);

        _motorController->SetForce(left, right);
    }

    void UserInputController::Enable()
    {
        _tcpServer->SetDataReceivedCallback([this](const std::vector<uint8_t>& data) { ReceiveMessage(data); });
    }

    void UserInputController::Disable()
    {
        _tcpServer->ResetDataReceivedCallback();
    }

} // Manhattan