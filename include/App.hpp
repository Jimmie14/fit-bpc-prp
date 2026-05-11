#pragma once

#include "Common/ConfigManager.hpp"

#include <rclcpp/rclcpp.hpp>

#include "Config/Configurable.hpp"
#include "Common/EventBus.hpp"
#include "Networking/TcpServer.h"
#include "Common/RosEngine.hpp"
#include <toml++/toml.hpp>

#include <any>

namespace Manhattan::core {

using namespace Manhattan::common;

class App {
public:
    const std::unique_ptr<EventBus> events;
    const std::shared_ptr<ConfigManager> config;

    App();

    std::shared_ptr<TcpServer> getTcpServer() const
    {
        return _tcpServer;
    }

    void run() const;

    template <typename T>
    requires std::is_base_of_v<RosComponent, T> std::shared_ptr<T> getComponent()
    const
    {
        const auto it = _components.find(typeid(T));
        if (it != _components.end()) {
            return std::static_pointer_cast<T>(it->second);
        }

        throw std::runtime_error("Component not found");
    }

    template <typename T, typename... Args>
    requires std::is_base_of_v<RosComponent, T> std::shared_ptr<T> addComponent(Args&&... args)
    {
        auto component = std::make_shared<T>(*this, std::forward<Args>(args)...);

        _components[typeid(T)] = component;
        _executor->add_node(component);

        return component;
    }

private:
    const std::shared_ptr<executors::MultiThreadedExecutor> _executor;
    std::shared_ptr<TcpServer> _tcpServer;

    std::unordered_map<std::type_index, std::shared_ptr<RosComponent>> _components;
};
} // namespace Manhattan::Core
