cmake_minimum_required(VERSION 3.22)
project(lbr_demos_advanced_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(FRIClient REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(lbr_fri_idl REQUIRED)
find_package(lbr_fri_ros2 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# admittance control node
add_library(admittance_control_component
  SHARED
    src/admittance_control_node.cpp
)

target_include_directories(admittance_control_component
  PRIVATE src
)

ament_target_dependencies(admittance_control_component
  kdl_parser
  lbr_fri_idl
  lbr_fri_ros2
  orocos_kdl_vendor
  rclcpp
  rclcpp_components
)

target_link_libraries(admittance_control_component
  FRIClient::FRIClient
)

rclcpp_components_register_node(admittance_control_component
  PLUGIN lbr_demos::AdmittanceControlNode
  EXECUTABLE admittance_control
)

# pose planning node
add_library(pose_planning_component
  SHARED
    src/pose_planning_node.cpp
)

target_include_directories(pose_planning_component
  PRIVATE src
)

ament_target_dependencies(pose_planning_component
  geometry_msgs
  rclcpp
  rclcpp_components
)

rclcpp_components_register_node(pose_planning_component
  PLUGIN lbr_demos::PosePlanningNode
  EXECUTABLE pose_planning
)

# pose contol node
add_library(pose_control_component
  SHARED
    src/pose_control_node.cpp
)

target_include_directories(pose_control_component
  PRIVATE src
)

ament_target_dependencies(pose_control_component
  geometry_msgs
  kdl_parser
  lbr_fri_idl
  orocos_kdl_vendor
  rclcpp
  rclcpp_components
)

target_link_libraries(pose_control_component
  FRIClient::FRIClient
)

rclcpp_components_register_node(pose_control_component
  PLUGIN lbr_demos::PoseControlNode
  EXECUTABLE pose_control
)

# install
install(
  TARGETS admittance_control_component pose_planning_component pose_control_component
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
