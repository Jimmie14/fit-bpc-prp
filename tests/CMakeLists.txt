find_package(GTest REQUIRED)
enable_testing()

macro(make_unit_test name)
    add_executable(${name} t_${name}.cpp)
    target_link_libraries(${name} GTest::GTest GTest::Main manhattan_libs robot_kinematics)
    ament_target_dependencies(${name} rclcpp std_msgs)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

make_unit_test(kinematics)