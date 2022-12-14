cmake_minimum_required(VERSION 3.16.3)
project(lbr_examples)

find_package(ament_cmake REQUIRED)

install(
    PROGRAMS
        scripts/admittance_control_node.py
        scripts/lbr_sinusoidal_node.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
