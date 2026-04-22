
# Root CMake file
cmake_minimum_required(VERSION 3.22)
project(project
        VERSION 1.2.0
        DESCRIPTION "" # todo: add description
        LANGUAGES CXX)

# Set C++ standard to C23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-g)

# packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)

# Setup ament package
ament_package()

# Add subdirectory
add_subdirectory(include)
add_subdirectory(src)
#add_subdirectory(tests)


install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)