#include "UserInputDriver.hpp"

#include "App.hpp"
#include "Kinematics.hpp"
#include "Networking/CommandParser.hpp"
using namespace std;
using namespace rclcpp;

namespace Manhattan::Core {
static std::vector<std::string> SplitBySpace(const std::string& value)
{
    std::istringstream iss(value);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

UserInputDriver::UserInputDriver(const App& app)
    : RosDeviceDriver(app, "user_input")
    , _kinematics(app.GetComponent<OdometryEngine>()->GetKinematics())
{
    _tcpServer = app.GetTcpServer();

    _motorController = app.GetComponent<MotorDriver>();
    _lineController = app.GetComponent<LineEngine>();

    Enable();
}

void UserInputDriver::ReceiveMessage(const std::vector<uint8_t>& data)
{
    std::istringstream iss(std::string(data.begin(), data.end()));
    std::string command;
    std::string value;

    iss >> command;
    std::getline(iss, value);

    // Remove leading space from value if present
    if (!value.empty() && value[0] == ' ')
        value.erase(0, 1);

    DecodeMessage(command, SplitBySpace(value));
}

void UserInputDriver::DecodeMessage(const std::string& command, const std::vector<std::string>& values)
{
    if (command == "STOP") {
        _motorController->SetForce(0, 0);
        if (_activeController)
            _activeController->Disable();

        _activeController.reset();
        return;
    }

    if (command == "MOVE") {
        ParseMoveCommand(values);
        return;
    }

    if (command == "MODE") {
        DecodeModeCommand(values);
        return;
    }

    if (command == "CFG" && values[0] == "LINE_FOLLOW") {
        ParseLineConfig(values);
    }
}

void UserInputDriver::DecodeModeCommand(const std::vector<std::string>& values)
{
    const auto mode = values[0];

    if (mode == "LINE_FOLLOW") {
        _lineController->Enable();
        _activeController = std::dynamic_pointer_cast<RosComponent>(_lineController);

        return;
    }

    if (mode == "CORRIDOR") {
    }
}

void UserInputDriver::ParseMoveCommand(const std::vector<std::string>& values) const
{
    const auto linear = CommandParser::ParseValue<double>("linear", values).value_or(0);
    const auto angular = CommandParser::ParseValue<double>("angular", values).value_or(0);

    const auto robotSpeed = RobotSpeed(linear, angular);
    auto [left, right] = _kinematics.inverse(robotSpeed);

    _motorController->SetForce(left, right);
}

void UserInputDriver::ParseLineConfig(const std::vector<std::string>& values) const
{
    const auto maxSpeed = CommandParser::ParseValue<double>("max_speed", values).value_or(0);
    _lineController->SetMaxSpeed(maxSpeed);

    const auto kp = CommandParser::ParseValue<double>("kp", values);
    const auto ki = CommandParser::ParseValue<double>("ki", values);
    const auto kd = CommandParser::ParseValue<double>("kd", values);

    auto& pid = _lineController->GetPid();

    if (kp.has_value())
        pid.SetKp(kp.value());

    if (ki.has_value())
        pid.SetKi(ki.value());

    if (kd.has_value())
        pid.SetKd(kd.value());
}

void UserInputDriver::OnEnable()
{
    _tcpServer->SetDataReceivedCallback([this](const std::vector<uint8_t>& data) { ReceiveMessage(data); });
}

void UserInputDriver::OnDisable()
{
    _tcpServer->ResetDataReceivedCallback();
}

} // namespace Manhattan::Core