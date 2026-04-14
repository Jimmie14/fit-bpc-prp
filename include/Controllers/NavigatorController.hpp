
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include "BaseController.h"
#include "SlamController.hpp"
#include "MotorController.hpp"


namespace Manhattan::Core {
    class NavigatorController final : public BaseController {
    public:
        explicit NavigatorController(const App& app);

        void SetPath(std::vector<GridCell*> path);
        bool HasPath() const;
        void ClearPath();

        std::vector<GridCell> CalculatePath(GridCell* destination) const; // todo: move this to somewhere else

    private:
        std::shared_ptr<MotorController> _motor; // todo: change naming of MotorController
        std::shared_ptr<SlamController> _slam;

        std::vector<GridCell*> _path;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pathPublisher;

        Vector2 GetDirection(const Pose &pose, const Vector2 &desiredDirection) const;

        void Update() const;
    };

}

