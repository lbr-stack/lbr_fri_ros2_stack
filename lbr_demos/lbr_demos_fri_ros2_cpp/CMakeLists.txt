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
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl_vendor)
find_package(kdl_parser REQUIRED)

# cartesian_pose_node
add_executable(cartesian_pose_node
  src/cartesian_pose_node.cpp
)

ament_target_dependencies(cartesian_pose_node
  fri_vendor
  lbr_fri_msgs
  rclcpp
  tf2_ros
  urdf
  sensor_msgs
  orocos_kdl_vendor
  kdl_parser
  geometry_msgs
)

target_link_libraries(cartesian_pose_node
  FRIClient::FRIClient
)

# cartesian_path_planning_node
add_executable(cartesian_path_planning_node
  src/cartesian_path_planning_node.cpp
)

ament_target_dependencies(cartesian_path_planning_node
  fri_vendor
  lbr_fri_msgs
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
)

target_link_libraries(cartesian_path_planning_node
  FRIClient::FRIClient
)

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
  cartesian_pose_node
  cartesian_path_planning_node
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
