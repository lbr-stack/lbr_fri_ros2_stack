cmake_minimum_required(VERSION 3.8)
project(lbr_fri_ros2_advanced_cpp_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lbr_fri_ros2 REQUIRED)
find_package(lbr_fri_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

add_executable(admittance_control_node src/admittance_control_node.cpp)
ament_target_dependencies(
  admittance_control_node
  Eigen3
  orocos_kdl
  kdl_parser
  rclcpp
  lbr_fri_ros2
  lbr_fri_msgs
)

install(
  TARGETS admittance_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
