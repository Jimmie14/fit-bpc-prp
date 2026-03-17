#pragma once
#include <rclcpp/rclcpp.hpp>

#include "Controllers/BaseController.h"
#include "Networking/TcpServer.h"

namespace Manhattan::Core
{
    class App {
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;
        std::unique_ptr<TcpServer> _tcpServer;

        std::vector<std::shared_ptr<BaseController>> _controllers;

        std::function<void()> _stopCallback;
        std::function<void(double linear, double angular)> _moveCallback;
    public:
        App();

        void SetStopCallback(const std::function<void()>& stopCallback) {
            _stopCallback = stopCallback;
        }
        void SetMoveCallback(const std::function<void(double linear, double angular)>& callback) {
            _moveCallback = callback;
        }

        void ReceiveMessage(const std::string &message) const;

        void Run() const;

        template<typename T, typename... Args>
        requires std::is_base_of_v<BaseController, T>
        std::shared_ptr<T> AddController(Args&&... args)
        {
            auto controller = std::make_shared<T>(std::forward<Args>(args)...);

            _controllers.push_back(controller);

            if (auto node = controller->GetNode()) {
                _executor->add_node(node);
            }

            return controller;
        }
    };
}
