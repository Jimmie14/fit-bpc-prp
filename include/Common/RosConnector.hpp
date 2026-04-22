#pragma once

#include <rclcpp/rclcpp.hpp>

namespace Manhattan::Core {
class App;

class RosConnector {
public:
    explicit RosConnector(const App& app);

    virtual ~RosConnector() = default;

    void Enable();

    void Disable();

    [[nodiscard]] virtual std::shared_ptr<rclcpp::Node> GetNode() const
    {
        return nullptr;
    }

protected:
    const App& _app;
    const rclcpp::Node::SharedPtr _node;

    virtual void OnEnable()
    {
    }

    virtual void OnDisable()
    {
    }

private:
    bool _enabled = false;
};
} // namespace Manhattan::Core
