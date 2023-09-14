cmake_minimum_required(VERSION 3.16)
project(lbr_fri_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(fri_vendor REQUIRED)
find_package(FRIClient REQUIRED)
find_package(lbr_fri_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)

# lbr_fri_ros2
add_library(lbr_fri_ros2
  SHARED
    src/app.cpp
    src/client.cpp
    src/command_guard.cpp
    src/utils.cpp
)

target_include_directories(lbr_fri_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lbr_fri_ros2
  control_toolbox
  lbr_fri_msgs
  rclcpp
  realtime_tools
  urdf
)

target_link_libraries(lbr_fri_ros2
  FRIClient::FRIClient
)

ament_export_targets(lbr_fri_ros2_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  control_toolbox
  fri_vendor
  FRIClient
  lbr_fri_msgs
  rclcpp
  realtime_tools
  urdf
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
  DIRECTORY launch
  DESTINATION share/lbr_fri_ros2
)

install(
  PROGRAMS scripts/lbr_state_smoothing_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch mixins
ament_python_install_package(${PROJECT_NAME})

ament_package()
