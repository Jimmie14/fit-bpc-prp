
#add_library(robot_kinematics Kinematics.cpp)
#target_link_libraries(robot_kinematics manhattan_libs)

file(GLOB_RECURSE PROJ_FILES CONFIGURE_DEPENDS "*.cpp")

add_executable(manhattan ${PROJ_FILES})
target_link_libraries(manhattan PUBLIC rclcpp::rclcpp manhattan_libs)



ament_target_dependencies(manhattan PUBLIC rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs visualization_msgs)

install(TARGETS manhattan DESTINATION lib/${PROJECT_NAME})
