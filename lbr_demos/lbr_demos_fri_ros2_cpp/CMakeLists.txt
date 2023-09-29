cmake_minimum_required(VERSION 3.22)
project(lbr_demos_fri_ros2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fri_vendor REQUIRED)
find_package(FRIClient REQUIRED)
find_package(lbr_fri_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# joint sine overlay
add_executable(joint_sine_overlay_node
  src/joint_sine_overlay_node.cpp
)

ament_target_dependencies(joint_sine_overlay_node
  fri_vendor
  lbr_fri_msgs
  rclcpp
)

target_link_libraries(joint_sine_overlay_node
  FRIClient::FRIClient
)
  
# torque sine overlay
add_executable(torque_sine_overlay_node
  src/torque_sine_overlay_node.cpp
)

ament_target_dependencies(torque_sine_overlay_node
  fri_vendor
  lbr_fri_msgs
  rclcpp
)

target_link_libraries(torque_sine_overlay_node
  FRIClient::FRIClient
)

# wrench sine overlay
add_executable(wrench_sine_overlay_node
  src/wrench_sine_overlay_node.cpp
)

ament_target_dependencies(wrench_sine_overlay_node
  fri_vendor
  lbr_fri_msgs
  rclcpp
)

target_link_libraries(wrench_sine_overlay_node
  FRIClient::FRIClient
)

install(TARGETS
  joint_sine_overlay_node
  torque_sine_overlay_node
  wrench_sine_overlay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
