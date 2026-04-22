#pragma once

#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace rclcpp;

namespace Manhattan::Core {
class App;

class RosComponent : public Node {
public:
    explicit RosComponent(const App& app, const string& nodeName);

    void Enable();

    void Disable();
protected:
    const App& _app;

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
