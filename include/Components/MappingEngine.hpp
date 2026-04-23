#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../Math/PoseMatcher.hpp"
#include "../Math/Vector2.hpp"
#include "App.hpp"
#include "OccupancyGrid.hpp"

namespace Manhattan::Core {
enum class MappingEngineState {
    Initializing,
    Stable,
    Degraded,
    Lost,
};

struct MappingEngineStateChangeEvent {
    MappingEngineState oldState;
    MappingEngineState newState;
};

class MappingEngine final : public RosEngine {
public:
    explicit MappingEngine(App& app);
    GridCell* GetCell(const Vector2& position);
    std::vector<GridCell*> GetNeighbors(const GridCell* cell);
    bool RayCast(const Vector2& worldPosition, const Vector2& direction, RayHit& rayHit, double maxDistance = 100);

    [[nodiscard]] Pose CurrentPose() const
    {
        return _stablePose;
    }

private:
    Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometrySub;

    TimerBase::SharedPtr _publishTimer;
    TimerBase::SharedPtr _costUpdateTimer;

    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _posePublisher;
    Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _hypoPublisher;
    Publisher<nav_msgs::msg::Path>::SharedPtr _pathPublisher;

    Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _gridPublisher;
    Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _gridMapPublisher;

    OccupancyGrid _grid;
    PoseMatcher _poseMatcher;

    Pose _lastOdomPose;
    Pose _odomPoseDelta = Pose::Zero();

    Pose _lastStoredPose = Pose::Identity();
    Pose _stablePose = Pose::Identity();
    std::vector<PoseMatchResult> _hypotheses;

    MappingEngineState _state = MappingEngineState::Initializing;
    Time _lostTime;

    std::vector<Vector2> _lastScan;

    nav_msgs::msg::Path _path;

    std::mutex _mapLock;
    std::mutex _odomLock;

    void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg);

    void OnLidar(const std::vector<Vector2>& points);

    void UpdateHypotheses(const Pose& odomDelta);
    void UpdateState();
    void ChangeState(MappingEngineState newState);

    void CreateHypothesis();

    void Reset();

    void MapScan(const std::vector<Vector2>& points);

    void Publish();

    void PublishPose();
    void PublishGrid();
    void PublishGridMap();
};
} // namespace Manhattan::Core
