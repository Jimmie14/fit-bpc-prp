#pragma once
#include <rclcpp/rclcpp.hpp>

namespace Manhattan::Core
{
    class App;

    class BaseController {
    protected:
        const App& _app;
        const rclcpp::Node::SharedPtr _node;
    public:
        explicit BaseController(const App& app);

        virtual ~BaseController() = default;

        virtual void Enable() { }

        virtual void Disable() { }

        [[nodiscard]] virtual std::shared_ptr<rclcpp::Node> GetNode() const { return nullptr; }
    };
}
