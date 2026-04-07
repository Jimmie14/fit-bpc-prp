#!/bin/bash

# move to parent
cd ..

# Build the workspace
colcon build || { echo "Build failed"; exit 1; }

# Source the ROS2 setup
source install/setup.bash

# Default launch options
LAUNCH_OPTIONS=""

# Parse arguments
for arg in "$@"; do
  if [ "$arg" == "--headless" -o "$arg" == "-h" ]; then
    LAUNCH_OPTIONS="launch_manhattan:=false"
  fi
done

# Run your ROS2 node with or without the headless option
ros2 launch project bringup.launch.py $LAUNCH_OPTIONS
