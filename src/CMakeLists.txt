
add_library(robot_kinematics Kinematics.cpp)
target_link_libraries(robot_kinematics manhattan_libs)

add_executable(manhattan main.cpp App.cpp Controllers/Node/NodeController.cpp Controllers/Node/MotorController.cpp Networking/TcpServer.cpp)
target_link_libraries(manhattan PUBLIC rclcpp::rclcpp manhattan_libs robot_kinematics)



ament_target_dependencies(manhattan PUBLIC rclcpp std_msgs)

install(TARGETS manhattan DESTINATION lib/${PROJECT_NAME})
