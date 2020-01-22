cmake_minimum_required(VERSION 3.10.2)
project(lbr_fri_ros
    LANGUAGES CXX
)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fri REQUIRED)

# Add targets
add_executable(lbr_fri_ros
    src/fri_ros.cpp
)

# Add include directories for target
target_include_directories(lbr_fri_ros
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link against dependencies
ament_target_dependencies(lbr_fri_ros rclcpp fri)

# Export target
ament_export_interfaces(export_lbr_fri_ros)

install(
    TARGETS lbr_fri_ros
    EXPORT export_lbr_fri_ros
    RUNTIME DESTINATION bin
)

ament_package()