#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "BaseController.h"
#include "Kinematics.hpp"
#include "SlamController.hpp"
#include "MotorController.hpp"
#include "Pid.hpp"

namespace Manhattan::Core {
    class NavigatorController final : public BaseController {
    public:
        explicit NavigatorController(const App& app);

        void SetPath(std::queue<GridCell*> path);
        [[nodiscard]] bool HasPath() const;
        void ClearPath();

        std::queue<GridCell*> CalculatePath(GridCell* destination) const; // todo: move this to somewhere else

    private:
        double _currentAngularVelocity = 0.0;
        double _currentLinearVelocity = 0.0;

        Kinematics _kinematics;
        Pid _angularPid;

        rclcpp::TimerBase::SharedPtr _timer;
        std::shared_ptr<MotorController> _motor; // todo: change naming of MotorController
        std::shared_ptr<SlamController> _slam;

        std::queue<GridCell*> _path;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pathPublisher;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _rayCastPublisher;

        std::vector<RayHit> RayCastAround(const Pose &pose) const;
        Vector2 GetDirection(const std::vector<RayHit> &rayHits, const Pose &pose, const Vector2 &desiredDirection) const;

        std::vector<GridCell*> SmoothPath(const std::vector<GridCell*>& pathList) const;
        bool HasLineOfSight(GridCell* start, GridCell* end) const;
        bool IsBlocking(const Vector2& position) const;

        void PublishPath() const;
        void PublishRayCast(const std::vector<RayHit> &hits, const Pose &pose, const Vector2 &desiredDirection) const;
        void Update();
    };

}

