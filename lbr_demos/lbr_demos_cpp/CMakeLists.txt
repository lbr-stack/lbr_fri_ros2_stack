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
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(std_srvs REQUIRED)

# joint sine overlay
add_executable(joint_sine_overlay
  src/joint_sine_overlay.cpp
)

ament_target_dependencies(joint_sine_overlay
  lbr_fri_idl
  lbr_fri_ros2
  rclcpp
)

target_link_libraries(joint_sine_overlay
  FRIClient::FRIClient
)

# joint trajectory client
add_executable(joint_trajectory_client
  src/joint_trajectory_client.cpp
)

ament_target_dependencies(joint_trajectory_client
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

target_link_libraries(joint_trajectory_client
  FRIClient::FRIClient
)

# torque sine overlay
add_executable(torque_sine_overlay
  src/torque_sine_overlay.cpp
)

ament_target_dependencies(torque_sine_overlay
  lbr_fri_idl
  lbr_fri_ros2
  rclcpp
)

target_link_libraries(torque_sine_overlay
  FRIClient::FRIClient
)

# wrench sine overlay
add_executable(wrench_sine_overlay
  src/wrench_sine_overlay.cpp
)

ament_target_dependencies(wrench_sine_overlay
  lbr_fri_idl
  lbr_fri_ros2
  rclcpp
)

target_link_libraries(wrench_sine_overlay
  FRIClient::FRIClient
)

# effort example controller
add_library(lbr_demos_cpp
  SHARED
  src/effort_example_controller.cpp
)

ament_target_dependencies(lbr_demos_cpp
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
  lbr_fri_idl
  lbr_fri_ros2
)

pluginlib_export_plugin_description_file(
  controller_interface effort_example_controller.xml)

install(TARGETS
  joint_sine_overlay
  joint_trajectory_client
  torque_sine_overlay
  wrench_sine_overlay
  lbr_demos_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
