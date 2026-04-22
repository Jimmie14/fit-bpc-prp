#pragma once

#include "../Common/RosConnector.hpp"
#include "LineController.hpp"
#include "Networking/TcpServer.h"
#include "OdometryEngine.hpp"

namespace Manhattan::Core {
class UserInputController final : public RosConnector {
    std::shared_ptr<TcpServer> _tcpServer;

    std::shared_ptr<MotorDriver> _motorController;
    std::shared_ptr<LineController> _lineController;

    std::shared_ptr<RosConnector> _activeController;

    Kinematics _kinematics;

public:
    explicit UserInputController(const App& app);

    void ReceiveMessage(const std::vector<uint8_t>& data);

    void OnEnable() override;

    void OnDisable() override;

private:
    void DecodeMessage(const std::string& command, const std::vector<std::string>& values);

    void DecodeModeCommand(const std::vector<std::string>& values);

    void ParseMoveCommand(const std::vector<std::string>& values) const;

    void ParseLineConfig(const std::vector<std::string>& values) const;
};
} // namespace Manhattan::Core