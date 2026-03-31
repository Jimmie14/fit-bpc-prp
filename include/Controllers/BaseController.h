#pragma once
#include <rclcpp/rclcpp.hpp>

namespace Manhattan::Core
{
    class App;

    class BaseController {
    private:
        bool _enabled = false;

    protected:
        const App& _app;
        const rclcpp::Node::SharedPtr _node;

        virtual void OnEnable() { }

        virtual void OnDisable() { }

    public:
        explicit BaseController(const App& app);

        virtual ~BaseController() = default;

        void Enable();

        void Disable();

        [[nodiscard]] virtual std::shared_ptr<rclcpp::Node> GetNode() const { return nullptr; }
    };
}
