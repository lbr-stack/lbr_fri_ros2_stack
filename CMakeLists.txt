cmake_minimum_required(VERSION 3.5)
project(fast_robot_interface_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_subdirectory(third_party/FastRobotInterface)

add_executable(frinode src/fri_node.cpp src/lbr_servo.cpp)
target_include_directories(frinode PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(frinode FRI::fri)
ament_target_dependencies(frinode rclcpp std_msgs)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

install(TARGETS
 frinode
 publisher
 DESTINATION lib/${PROJECT_NAME})

ament_package()