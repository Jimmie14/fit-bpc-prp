#pragma once
#include <rclcpp/rclcpp.hpp>

#include "EventBus.hpp"
#include "Networking/TcpServer.h"
#include "RosDeviceDriver.hpp"
#include "RosEngine.hpp"

namespace Manhattan::Core {
class App {
public:
    const std::unique_ptr<EventBus> Events;

    App();

    std::shared_ptr<TcpServer> GetTcpServer() const
    {
        return _tcpServer;
    }

    void Run() const;

    template <typename T>
    requires std::is_base_of_v<RosComponent, T> std::shared_ptr<T> GetComponent()
    const
    {
        auto it = _components.find(typeid(T));
        if (it != _components.end()) {
            return std::static_pointer_cast<T>(it->second);
        }

        return nullptr;
    }

    template <typename T, typename... Args>
    requires std::is_base_of_v<RosComponent, T> std::shared_ptr<T> AddComponent(Args&&... args)
    {
        auto component = std::make_shared<T>(*this, std::forward<Args>(args)...);

        _components[typeid(T)] = component;
        _executor->add_node(component);

        return component;
    }

    template <typename T, typename... Args>
    requires std::is_base_of_v<RosDeviceDriver, T> std::shared_ptr<T> AddDriver(Args&&... args)
    {
        return AddComponent<T>(std::forward<Args>(args)...);
    }

    template <typename T, typename... Args>
    requires std::is_base_of_v<RosEngine, T> std::shared_ptr<T> AddEngine(Args&&... args)
    {
        return AddComponent<T>(std::forward<Args>(args)...);
    }

private:
    const std::shared_ptr<executors::MultiThreadedExecutor> _executor;
    std::shared_ptr<TcpServer> _tcpServer;

    std::unordered_map<std::type_index, std::shared_ptr<RosComponent>> _components;
};
} // namespace Manhattan::Core
