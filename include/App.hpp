#pragma once
#include "Controller.hpp"

using namespace std;

namespace Manhattan::Core
{
    class App {
        shared_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;

        vector<shared_ptr<BaseController>> _controllers;

    public:
        App() {
            _executor = make_shared<rclcpp::executors::MultiThreadedExecutor>();
        }

        void Run() {
            _executor->spin();
        }

        template<typename T>
        requires is_base_of_v<BaseController, T>
        shared_ptr<T> AddController() {
            auto nodeName = std::string(T::TypeName) + "_" + std::to_string(_controllers.size());
            auto node = make_shared<rclcpp::Node>(nodeName);
            auto controller = make_shared<T>(node);

            _controllers.push_back(controller);
            _executor->add_node(node);

            return controller;
        }
    };
}