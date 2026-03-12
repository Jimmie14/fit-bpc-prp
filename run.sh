#!/bin/bash

# Build the workspace
colcon build || { echo "Build failed"; exit 1; }

# Source the ROS2 setup
source install/setup.bash

# Run your ROS2 node
ros2 run project manhattan