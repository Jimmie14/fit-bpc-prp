#pragma once
#include <rclcpp/rclcpp.hpp>

#include "Controllers/BaseController.h"
#include "Networking/TcpServer.h"

namespace Manhattan::Core
{
    class App {
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;
        std::shared_ptr<TcpServer> _tcpServer;

        std::unordered_map<std::type_index, std::shared_ptr<BaseController>> _controllers;

        rclcpp::Node::SharedPtr _node;
    public:
        App();

        std::shared_ptr<TcpServer> GetTcpServer() const { return _tcpServer; }

        void Run() const;

        rclcpp::Node::SharedPtr GetNode() const { return _node; }

        template<typename T>
        requires std::is_base_of_v<BaseController, T>
        std::shared_ptr<T> GetController() const
        {
            auto it = _controllers.find(typeid(T));
            if (it != _controllers.end()) {
                return std::static_pointer_cast<T>(it->second);
            }

            return nullptr;
        }

        template<typename T, typename... Args>
        requires std::is_base_of_v<BaseController, T>
        std::shared_ptr<T> AddController(Args&&... args)
        {
            auto controller = std::make_shared<T>(*this, std::forward<Args>(args)...);
            _controllers[typeid(T)] = controller;

            return controller;
        }
    };
}
