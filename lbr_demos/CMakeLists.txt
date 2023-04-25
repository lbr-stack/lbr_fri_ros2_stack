cmake_minimum_required(VERSION 3.16.3)
project(lbr_demos)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lbr_fri_msgs REQUIRED)

add_executable(
    lbr_command_node
    src/lbr_command_node.cpp
)

ament_target_dependencies(
    lbr_command_node
    rclcpp
    lbr_fri_msgs
)

install(
    PROGRAMS
        scripts/admittance_control_node.py
        scripts/lbr_command_node.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS lbr_command_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
