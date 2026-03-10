#pragma once

using namespace std;

namespace Manhattan::Core
{
    class BaseController {
    protected:
        rclcpp::Time _startTime;
        rclcpp::Node::SharedPtr _node;

    public:
        explicit BaseController(const rclcpp::Node::SharedPtr &node)
            : _node(node)
        {
            _startTime = _node->now();
        }
        virtual ~BaseController() = default;
    };
}