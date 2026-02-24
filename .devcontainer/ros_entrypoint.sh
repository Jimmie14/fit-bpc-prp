#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -euo pipefail

echo "Hello world!"
echo "test" >> "$HOME/ros2_ws/entrypoint.log"

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"

exec "$@"