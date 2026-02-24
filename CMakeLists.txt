
# Root CMake file
cmake_minimum_required(VERSION 3.22)
project(bcp_prp
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

# Enable testingcm
option(ENABLE_TESTING "Build tests" ON)

if (ENABLE_TESTING)
    enable_testing()
    include(CTest)
endif()

add_subdirectory(include)
add_subdirectory(src)
