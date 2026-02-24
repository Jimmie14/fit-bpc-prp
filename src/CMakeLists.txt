
add_executable(manhattan main.cpp)
target_link_libraries(manhattan PRIVATE rclcpp::rclcpp)


install(TARGETS
        manhattan
        DESTINATION lib/${PROJECT_NAME})

