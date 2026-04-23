#pragma once

#include "RosEngine.hpp"
#include "NavigatorEngine.hpp"
#include "MappingEngine.hpp"
#include "NavigatorGraphBuilder.hpp"

namespace Manhattan::Core {

class FollowerEngine : public RosEngine {
public:
    explicit FollowerEngine(const App& app);

    void OnEnable() override;
    void OnDisable() override;

private:
    void Update();
    void FollowCorridor();

    std::shared_ptr<MappingEngine>_map;
    std::shared_ptr<NavigatorEngine> _navigator;
    std::shared_ptr<NavigatorGraphBuilder> _graphBuilder;

    std::shared_ptr<NavigatorNode> _currentNode;
    std::shared_ptr<Edge> _currentEdge;

    TimerBase::SharedPtr _timer;
    TimerBase::SharedPtr _initialTimer;
};

} // namespace Manhattan::Core