
#add_library(robot_kinematics Kinematics.cpp)
#target_link_libraries(robot_kinematics manhattan_libs)

file(GLOB_RECURSE PROJ_FILES CONFIGURE_DEPENDS "*.cpp")

add_executable(manhattan ${PROJ_FILES})
target_link_libraries(manhattan PUBLIC rclcpp::rclcpp manhattan_libs tomlplusplus::tomlplusplus)

ament_target_dependencies(manhattan PUBLIC
        rclcpp
        std_msgs nav_msgs geometry_msgs
        sensor_msgs visualization_msgs
        grid_map_msgs grid_map_ros
        tf2
        OpenCV
)

add_custom_target(copy_config ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_SOURCE_DIR}/config/manhattan.toml
    ${CMAKE_CURRENT_BINARY_DIR}
)
add_dependencies(manhattan copy_config)

install(TARGETS manhattan DESTINATION lib/${PROJECT_NAME})
