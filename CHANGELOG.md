## [unreleased]

### 🚜 Refactor

- Major architecture refactor
- Rename SlamController to MappingEngine

### 🎨 Styling

- Run clang-format

### ⚙️ Miscellaneous Tasks

- Correct skip rule for 'ci: update changelog' in cliff config
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
