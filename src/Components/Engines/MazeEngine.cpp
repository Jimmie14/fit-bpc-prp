// #include "MazeEngine.hpp"
//
// using namespace std;
//
// namespace Manhattan::Core {
//
// MazeEngine::MazeEngine(const App& app)
//     : RosEngine(app, "maze")
// {
//     _navigator = app.GetComponent<NavigatorEngine>();
//     _mapping = app.GetComponent<MappingEngine>();
// }
//
// void MazeEngine::OnEnable() {
//     _initialTimer = create_wall_timer(1s, [this]() {
//         _initialTimer->cancel();
//         _timer = create_wall_timer(100ms, [this] { Update(); });
//     });
// }
//
// void MazeEngine::OnDisable() {
//     _timer.reset();
//     _initialTimer.reset();
// }
//
// void MazeEngine::Update() {
//     if (!_navigator->IsInDestination())
//         return;
//
//     if (_currentWayPoint == nullptr) {
//         _currentWayPoint = std::make_shared<WayPoint>();
//
//         _currentWayPoint->position = _mapping->CurrentPose().position;
//         _currentWayPoint->connected = {};
//         _currentWayPoint->visited = true;
//     }
//
//     const auto target = NextJunction(_currentWayPoint);
//     if (target == nullptr)
//         return;
//
//     _currentWayPoint->visited = true;
//     _currentWayPoint = target;
//
//     const auto cell = _mapping->GetCell(target->position);
//     _navigator->SetDestination(cell);
// }
//
// std::shared_ptr<MazeEngine::WayPoint> MazeEngine::NextJunction(std::shared_ptr<WayPoint> current) {
//     std::vector<bool> thinned_map; // todo get from ZhangSuenThinning
//
//     std::queue<Vector2Int> q;
//     std::set<Vector2Int> visited;
//
//     q.push(_mapping->WorldToGrid(current->position)); // todo, find closes point in thinned graph
//     visited.insert(q.front());
//
//     while (!q.empty()) {
//         auto cell = q.front(); q.pop();
//         int traversable_neighbors = 0;
//         std::vector<Vector2Int> neighbors;
//     }
//
//     for (auto direction : Vector2Int::Directions()) {
//         auto neighbour = thinned_map[cell + direction];
//
//
//     }
// }
//
// } // namespace Manhattan::Core