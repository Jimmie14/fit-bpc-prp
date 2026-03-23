#pragma once
#include <rclcpp/rclcpp.hpp>

namespace Manhattan::Core
{
    class App;

    class BaseController {
    protected:
        const App& _app;
        const rclcpp::Node::SharedPtr _node;

        bool _enabled = false;
    public:
        explicit BaseController(const App& app);

        virtual ~BaseController() = default;

        void Enable() {
            _enabled = true;
        }

        void Disable() {
            _enabled = false;
        }

        [[nodiscard]] virtual std::shared_ptr<rclcpp::Node> GetNode() const { return nullptr; }
    };
}
