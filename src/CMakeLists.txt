
add_library(robot_kinematics kinematics.cpp)
target_link_libraries(robot_kinematics manhattan_lib)

add_executable(manhattan main.cpp)
target_link_libraries(manhattan PUBLIC rclcpp::rclcpp manhattan_lib robot_kinematics)



ament_target_dependencies(manhattan PUBLIC rclcpp std_msgs)

install(TARGETS manhattan DESTINATION lib/${PROJECT_NAME})
