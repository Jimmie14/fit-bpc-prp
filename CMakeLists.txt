
# Root CMake file
cmake_minimum_required(VERSION 3.28)
project(BPC_PRP
        VERSION 1.0
        DESCRIPTION "" # todo: add description
        LANGUAGES CXX)

# Set C++ standard to C23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable testing
option(ENABLE_TESTING "Build tests" ON)

if (ENABLE_TESTING)
    enable_testing()
    include(CTest)
endif()

add_subdirectory(libs)
add_subdirectory(projects)
# add_subdirectory(tests)
