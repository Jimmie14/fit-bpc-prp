[![C++](https://img.shields.io/badge/C%2B%2B-23-blue?logo=cplusplus)](https://isocpp.org/)
[![ROS](https://img.shields.io/badge/ROS-Humble-green?logo=ros)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

# Project Manhattan

A school project for controlling robot demonstrating autonomous navigation capabilities.

## Table Of Contents
- [🚀 Quick Start](#-quick-start)
- [📋 Code Conventions](#-code-conventions)
- [📄 License](#-license)

## 🚀 Quick Start
Follow these steps to set up the development environment on macOS:

### Prerequisites
- Ubuntu 22.04 Live Server
- ROS Humble
- JetBrains CLion or compatible C++ IDE

### Environment Setup
```bash
# Add ROS to your environment (.bashrc/.profile/...)
source /opt/ros/humble/setup.bash

mkdir ~/ros_ws
cd ~/ros_ws

git clone <project-url> project
# for more information see ./env-setup.sh
```

### Building
```bash
# Run project with standard way
# this will build the project, source it and run all
# required dependencies (like odometry)
./dev.sh run

# Run project in headless mode
# this will only run dependencies - it is useful
# for debugging or just run it once and use play/debug in CLion
./dev.sh --headless
```

### CLion Setup
1. Install **JetBrains Toolbox** and launch **JetBrains Gateway**.
2. Settings → Advanced Settings
   - Under SSH, set a custom path to the OpenSSH tool.
   - Use `ssh -l` to launch the backend so ROS sourcing works.
3. New Project → SSH → Enter Ubuntu VM credentials
4. Select `~/ros_ws/project` as the project path.

## 📋 Code Conventions
This project follows the **WebKit C++ Code Style**.

### Formatting
To automatically format all code files according to project run
```bash
./dev.sh format
```

## 📄 License

This project is licensed under the [MIT LIcense](LICENSE).

© 2025 xpitonm00, xpoulsa00.
