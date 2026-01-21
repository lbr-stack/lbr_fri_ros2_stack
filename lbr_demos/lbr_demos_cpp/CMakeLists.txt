cmake_minimum_required(VERSION 3.22)
project(lbr_demos_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(FRIClient REQUIRED)
find_package(lbr_fri_idl REQUIRED)
find_package(lbr_fri_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# joint sine overlay
add_executable(joint_sine_overlay
  src/joint_sine_overlay.cpp
)

target_link_libraries(joint_sine_overlay PUBLIC
  FRIClient::FRIClient
  ${lbr_fri_idl_TARGETS}
  lbr_fri_ros2::lbr_fri_ros2
  rclcpp::rclcpp
)

# joint trajectory client
add_executable(joint_trajectory_client
  src/joint_trajectory_client.cpp
)

target_link_libraries(joint_trajectory_client PUBLIC
  ${control_msgs_TARGETS}
  FRIClient::FRIClient
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${sensor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
)

# torque sine overlay
add_executable(torque_sine_overlay
  src/torque_sine_overlay.cpp
)

target_link_libraries(torque_sine_overlay PUBLIC
  FRIClient::FRIClient
  ${lbr_fri_idl_TARGETS}
  lbr_fri_ros2::lbr_fri_ros2
  rclcpp::rclcpp
)

# wrench sine overlay
add_executable(wrench_sine_overlay
  src/wrench_sine_overlay.cpp
)

target_link_libraries(wrench_sine_overlay PUBLIC
  FRIClient::FRIClient
  ${lbr_fri_idl_TARGETS}
  lbr_fri_ros2::lbr_fri_ros2
  rclcpp::rclcpp
)

install(TARGETS
  joint_sine_overlay
  joint_trajectory_client
  torque_sine_overlay
  wrench_sine_overlay
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
