#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "App.hpp"
#include "OccupancyGrid.hpp"
#include "PoseMatcher.hpp"
#include "Vector2.hpp"

namespace Manhattan::Core {
enum class MappingEngineState {
    Initializing,
    Stable,
    Degraded,
    Lost,
};

struct ScanContext {
    static constexpr int ringCount = 20;
    static constexpr int sectorCount = 60;

    std::array<std::array<float, sectorCount>, ringCount> descriptor;
    Pose pose;

    [[nodiscard]] size_t GetHash() const
    {
        // coarse hash
        size_t hash = 0;

        for (auto r = 0; r < ringCount; ++r) {
            auto ringSum = 0.0f;
            auto ringMax = 0.0f;

            for (auto s = 0; s < sectorCount; ++s) {
                ringSum += descriptor[r][s];
                ringMax = std::max(ringMax, descriptor[r][s]);
            }

            const auto quantizedSum = static_cast<int>(ringSum * 10) % 256;
            const auto quantizedMax = static_cast<int>(ringMax * 10) % 256;

            hash ^= (quantizedSum << (r % 8)) ^ (quantizedMax << ((r + 4) % 8));
        }

        return hash;
    }
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometrySub;

    rclcpp::TimerBase::SharedPtr _publishTimer;
    rclcpp::TimerBase::SharedPtr _costUpdateTimer;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _scanPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _posePublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pathPublisher;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _gridPublisher;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _gridMapPublisher;

    OccupancyGrid _grid;
    PoseMatcher _poseMatcher;

    Pose _lastOdomPose;
    Pose _odomPoseDelta = Pose::Zero();

    Pose _lastStoredPose = Pose::Identity();
    Pose _stablePose = Pose::Identity();
    std::vector<PoseMatchResult> _hypotheses;
    std::unordered_map<size_t, ScanContext> _scanContexts;

    MappingEngineState _state = MappingEngineState::Initializing;
    rclcpp::Time _lostTime;

    std::vector<Vector2> _lastScan;

    nav_msgs::msg::Path _path;

    std::mutex _mapLock;
    std::mutex _odomLock;

    void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg);

    void OnLidar(const std::vector<Vector2>& points);

    void UpdateHypotheses(const Pose& odomDelta);
    void UpdateState();

    void TryRelocalize();
    void StoreScanContext();
    static ScanContext ComputeScanContext(const std::vector<Vector2>& points, const Pose& pose);
    static double ComputeScanContextSimilarity(const ScanContext& a, const ScanContext& b);
    static Pose EstimatePoseFromScanContext(const ScanContext& current, const ScanContext& reference);

    void Reset();

    void MapScan(const std::vector<Vector2>& points);

    void Publish();

    void PublishScan() const;
    void PublishPose() const;

    void PublishGrid();
    void PublishGridMap();
};
} // namespace Manhattan::Core
