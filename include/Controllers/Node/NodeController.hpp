
#pragma once
#include "../BaseController.h"


namespace Manhattan::Core
{
    class NodeController : public BaseController
    {
        static unsigned int instance_count;

        protected:
            rclcpp::Node::SharedPtr _node;

        public:
            explicit NodeController(const std::string& nodeName);

            ~NodeController() override { instance_count--; }

            [[nodiscard]] std::shared_ptr<rclcpp::Node> GetNode() const override { return _node; }
    };
}
