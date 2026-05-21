#!/bin/bash

set -e

# Help function
help() {
    echo "Usage: $0 {run|format} [options]"
    exit 1
}

run() {
    # move to parent
    cd ..

    # Build the workspace
    colcon build || { echo "Build failed"; exit 1; }

    # Source the ROS2 setup
    source install/setup.bash

    LAUNCH_OPTIONS=""

    for arg in "$@"; do
        if [ "$arg" == "--headless" ] || [ "$arg" == "-h" ]; then
            LAUNCH_OPTIONS="launch_manhattan:=false"
        fi
    done

    ros2 launch project bringup.launch.py $LAUNCH_OPTIONS
}

format() {
    # Find all C/C++ source/header files and format them
    find . \( -path './src/*' -o -path './include/*' \) \( -name '*.cpp' -o -name '*.hpp' -o -name '*.c' -o -name '*.h' \) -print0 |
    while IFS= read -r -d '' file; do
        echo "Formatting $file"
        clang-format -i -style=file "$file" || {
            echo "Failed on $file"
            return 1
        }
    done

    echo "Formatting complete."
}

docs() {
    ./docs/build.sh
    cp ./docs/main.pdf ./paper.pdf
}

# Ensure a subcommand is provided
if [ $# -lt 1 ]; then
  usage
fi

SUBCOMMAND="$1"
shift


case "$SUBCOMMAND" in
  run)
    run "$@"
    ;;
  format)
    format "$@"
    ;;
  *)
    usage
    ;;
esac