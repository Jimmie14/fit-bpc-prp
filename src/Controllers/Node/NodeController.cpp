
#include "Controllers/Node/NodeController.hpp"

using namespace std;
using namespace rclcpp;

namespace Manhattan::Core
{
    unsigned int NodeController::instance_count = 0;

    NodeController::NodeController(const std::string& nodeName)
    {
        auto uniqueName = string(nodeName) + "_" + to_string(instance_count);

        _node = make_shared<Node>(uniqueName);

        instance_count++;
    }
}
