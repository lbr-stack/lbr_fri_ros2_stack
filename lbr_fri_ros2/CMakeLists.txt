cmake_minimum_required(VERSION 3.22)
project(lbr_fri_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(fri_vendor REQUIRED)
find_package(FRIClient REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(lbr_fri_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)

# lbr_fri_ros2
add_library(lbr_fri_ros2
  SHARED
    src/app.cpp
    src/async_client.cpp
    src/command_guard.cpp
    src/command_interface.cpp
    src/filters.cpp
    src/ft_estimator.cpp
    src/state_interface.cpp
)

target_include_directories(lbr_fri_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lbr_fri_ros2
  control_toolbox
  Eigen3
  kdl_parser
  lbr_fri_msgs
  orocos_kdl_vendor
  rclcpp
  realtime_tools
)

target_link_libraries(lbr_fri_ros2
  FRIClient::FRIClient
)

ament_export_targets(lbr_fri_ros2_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  control_toolbox
  eigen3_cmake_module
  Eigen3
  fri_vendor
  FRIClient
  kdl_parser
  lbr_fri_msgs
  orocos_kdl_vendor
  rclcpp
  realtime_tools
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lbr_fri_ros2
  EXPORT lbr_fri_ros2_export
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/lbr_fri_ros2
)

# app_component
add_library(app_component
  SHARED
    src/app_component.cpp
)

target_include_directories(app_component
  PRIVATE src
)

ament_target_dependencies(app_component
  rclcpp
  urdf
  rclcpp_components
)

target_link_libraries(app_component
  lbr_fri_ros2
)

rclcpp_components_register_node(app_component
  PLUGIN lbr_fri_ros2::AppComponent
  EXECUTABLE app
)

install(
  TARGETS app_component
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/lbr_fri_ros2
)

install(
  DIRECTORY config launch
  DESTINATION share/lbr_fri_ros2
)

# Launch mixins
ament_python_install_package(${PROJECT_NAME})

ament_package()
