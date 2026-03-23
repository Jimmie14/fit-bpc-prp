#pragma once

#include "BaseController.h"
#include "LineController.hpp"
#include "RobotOdometry.hpp"
#include "Networking/TcpServer.h"

namespace Manhattan::Core
{
    class UserInputController final : public BaseController {
        std::shared_ptr<TcpServer> _tcpServer;

        std::shared_ptr<MotorController> _motorController;
        std::shared_ptr<LineController> _lineController;

        std::shared_ptr<BaseController> _activeController;

        Kinematics _kinematics;

        bool _stopped;

    public:
        explicit UserInputController(const App& app);

        void ReceiveMessage(const std::vector<uint8_t>& data);

        void Enable() override;

        void Disable() override;

    private:
        void DecodeMessage(const std::string& command, const std::string& value);

        void ParseMoveCommand(const std::string& value) const;
    };
} // Manhattan::Core