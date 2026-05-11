## [1.6.0] - 2026-05-11

### 🚀 Features

- Use aru code in explorer
- Aru code applying on junctions
- Enhance navigation with reverse speed and distance calculations
- Add reverse mode
- Add stable version of navigator and explorer
- Add correct odometry linear and angular velocity detection
- Add pid regulator for each motor
- Minor dwpp cost function adjustments
- Add maze graph engine

### 🐛 Bug Fixes

- Fix all x-axis forward bugs
- Multiple node creation for single crossroad in maze graph engine

### 🚜 Refactor

- Change Pose to theta instead of rotation
- Major kinematics refactor

### ⚙️ Miscellaneous Tasks

- Removed unused constants
- Bump version to 1.6.0
## [1.5.1] - 2026-05-06

### 🐛 Bug Fixes

- Compilation bug in NavigatorEngine

### ⚙️ Miscellaneous Tasks

- Removed unused constants
- Bump version to 1.5.1
## [1.5.0] - 2026-05-06

### 🐛 Bug Fixes

- Bugs in NavigatorEngine

### ⚙️ Miscellaneous Tasks

- Bump version to 1.5.0

### ◀️ Revert

- ExplorerEngine GetCrossroadWays to previous version
## [1.4.1] - 2026-05-06

### 🚀 Features

- Add Grid.hpp abstraction
- Add MapThinning node
- Rename FollowerEngine to MazeEngine and add WorldToGrid conversion method
- Minor Grid improvements
- Refactor MazeEngine and add pose/map subscriptions with thinned map handling
- Enhance MazeEngine with waypoint graph publishing and traversal logic
- Integrate MazeEngine into the application and improve waypoint handling
- Enhance ExplorerEngine with improved pathfinding and map handling
- Add aruco detected event
- Add buttons driver allowing to reset map
- Refactor Explore method to return structured results and improve pathfinding logic
- Add explorer debug topic
- Add wall thicking before zhuang suent thinning
- Enhance picking algorithm for direction in crossroad
- Changed spline to linear
- Improved exploring logic to go from junction to junction
- Subscription to aru codes
- Navigator better turning

### 🐛 Bug Fixes

- GraphBuilder bugs
- Minor bugs
- Adjust pathfinding logic to handle neighbor count and update timer functionality
- Major Explorer engine fixes
- Navigator better handling
- Fix explorer bug on straight lines considering it crossroad
- Minor explorer fixes

### 🚜 Refactor

- Minor Grid refactor
- Major viz refactor

### ⚙️ Miscellaneous Tasks

- Bump version to 1.4.1

### ◀️ Revert

- FollowerEngine to use GraphBuilder
## [1.4.0] - 2026-04-27

### 🚀 Features

- Add ArucoDetectionEngine
- Enhance FollowerEngine with ray casting visualization and target adjustment logic
- Add aruco marker topic

### ⚙️ Miscellaneous Tasks

- Tune camera matrix and coefficients
- Bump version to 1.4.0
## [1.3.2] - 2026-04-27

### 🚀 Features

- Enhance FollowerEngine with improved target acquisition and navigation logic
- Improve hypothesis management and pose matching in MappingEngine

### ⚙️ Miscellaneous Tasks

- Bump version to 1.3.2
## [1.3.1] - 2026-04-23

### 🚀 Features

- Update NavigatorGraphBuilder to use NavigatorNode and improve graph construction
- Integrate NavigatorGraphBuilder into FollowerEngine for enhanced navigation
- Add state change handling in NavigatorEngine and update angular PID tuning

### ⚙️ Miscellaneous Tasks

- Bump version to 1.3.1
## [1.3.0] - 2026-04-22

### 🚀 Features

- Add Fast DDS support and optimize OccupancyGrid traversal
- Enable Cyclone DDS, adjust ROS node options, and optimize QoS settings
- Add monte carlo localization in MappingEngine

### ⚙️ Miscellaneous Tasks

- Bump version to 1.3.0
## [1.2.0] - 2026-04-22

### 🚀 Features

- Implement NavigatorGraphBuilder for pathfinding graph construction
- Add multiple hypothesis to MappingEngine

### 🐛 Bug Fixes

- [**breaking**] Migrate to multi-node architecture, fix performance bottleneck

### 🚜 Refactor

- Major architecture refactor
- Rename SlamController to MappingEngine
- Enhance navigation logic and smooth path calculation in NavigatorController
- Adjust QoS settings

### 🎨 Styling

- Run clang-format

### ⚙️ Miscellaneous Tasks

- Correct skip rule for 'ci: update changelog' in cliff config
- Bump version to 1.2.0
## [1.1.1] - 2026-04-22

### 🚀 Features

- Improve pose matching confidence response in SlamController
- Add ImuComponent republishing imu with covariance and correct frame
- Add missing lock bofere cost recalculation

### 🐛 Bug Fixes

- Recalculate cost in separete thread
- Recalculate cost in separate thread

### 🚜 Refactor

- Replace path queue with vector in ExplorerController and NavigatorController
- Rename HasPath to IsInDestination and update related logic
- Adjust speed parameters and improve time delta calculation in NavigatorController
- Clean up includes and add imuOrientationCovariance constant in ImuComponent
- Adjust navigation parameters and improve path evaluation logic
- Adjust ray count and distance parameters in FollowerController and NavigatorController

### ⚙️ Miscellaneous Tasks

- Bump version to 1.1.1
## [1.1.0] - 2026-04-20

### 🚀 Features

- Lidar controller
- Add ekf position filtering
- Add covariance table for robot odometry
- Line filtration
- Lidar scan filtration
- Pose matcher
- Add SlamController
- Protect update with mutex
- Add path topic to SlamController
- Add grid_map topic to SlamController
- Add NavigatorController.cpp
- Finish NavigatorController.cpp
- Follower controller
- Explorer controller
- Add desiredDirection publish to NavigatorController
- Add pid regulator for angular speec in NavigatorController
- Minor adjustments
- Path smooth
- Add corner factor to NavigatorController
- Add PoseMatcher confidence score

### 🐛 Bug Fixes

- SlamController topic publish
- SlamController rotation visualization
- Fix Pose bugs
- Minor fixes
- Slam controller publishing too often
- Minor slam fixes
- PublishRayCast invalid signature
- SlamController compilation bug

### 📚 Documentation

- Update README.md

### ⚙️ Miscellaneous Tasks

- Change default launch parameters
- Fixed sleep bug
- Run clang-format
- Add generate CHANGELOG workflow
- *(cliff)* Update cliff config to ignore save commits
- Bump version to 1.1.0
## [1.0.0] - 2026-03-24

### 🚀 Features

- Add burny project
- *(kinematics)* Implement forward and inverse kinematics for speed
- Motor controller
- Tcp server
- Communication implementation
- Add kinematics
- Line estimation
- Add path odometry
- Enhance line estimation with normalization and improved motor control
- Add pid line controller
- User input mode
- Implement command parsing and line controller configuration
- Minor line estimator improvements
- Add continuous line estimation combined with discrete approach
- Update PID parameters and adjust line threshold for improved line estimation

### 🐛 Bug Fixes

- BASH_ENV for non-interactive processes
- .gitignore not excluding ros2 files
- Project build
- Odometry bugs
- Odometry having no rate

### 🚜 Refactor

- Minor refacto
- Project structure
- Minor odometry refactor
- Remove node controller
- Minor LineController refactor
- Minor LineEstimator refactor

### 🧪 Testing

- Kinematics tests
- Add test

### ⚙️ Miscellaneous Tasks

- Add initial project
- Add git-cliff configuration
- Reorganized project to proper structure
- Delete .devcontainer directory
- Add devcontainer
- Fix path for copying ros_env.sh in Dockerfile
- Fix Dockerfile
- Add ssh-tools in dockerfile
- Change permission command to use sudo
- Remove chmod command for ros_env.sh
- Fix approach of ros entrypoint
- Fix approach of ros entrypoint
- Final devcontainer fixes
- Add README.md
- Minor project reorganization
- Better continues calibration
- Gamma correction
