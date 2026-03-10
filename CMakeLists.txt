
# Root CMake file
cmake_minimum_required(VERSION 3.22)
project(project
        VERSION 1.0
        DESCRIPTION "" # todo: add description
        LANGUAGES CXX)

# Set C++ standard to C23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Setup ament package
ament_package()

# Add subdirectory
add_subdirectory(include)
add_subdirectory(src)
# add_subdirectory(tests)