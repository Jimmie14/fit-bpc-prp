#pragma once
#include <rclcpp/rclcpp.hpp>

namespace Manhattan::Core
{
    class BaseController
    {
        public:
            virtual ~BaseController() = default;

            [[nodiscard]] virtual std::shared_ptr<rclcpp::Node> GetNode() const { return nullptr; }
    };
}
