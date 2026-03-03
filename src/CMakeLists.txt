
add_executable(manhattan main.cpp)
target_link_libraries(manhattan PUBLIC rclcpp::rclcpp manhattan_lib)

ament_target_dependencies(manhattan PUBLIC rclcpp std_msgs)

install(TARGETS manhattan DESTINATION lib/${PROJECT_NAME})
