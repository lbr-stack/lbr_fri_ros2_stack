cmake_minimum_required(VERSION 3.16)
project(lbr_fri_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(fri REQUIRED)
find_package(lbr_fri_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urdf REQUIRED)

# Add fri
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../fri/fri)

# lbr_fri_ros2
add_library(lbr_fri_ros2
  SHARED 
    src/lbr_client.cpp
    src/lbr_command_guard.cpp
    src/lbr_intermediary.cpp
)

target_include_directories(lbr_fri_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lbr_fri_ros2
      # fri
  lbr_fri_msgs
  urdf
)

target_link_libraries(${PROJECT_NAME} PRIVATE fri)

ament_export_targets(lbr_fri_ros2_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  # fri
  lbr_fri_msgs
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

# lbr_app_component
add_library(lbr_app_component
  SHARED
    src/lbr_app.cpp
)

target_include_directories(lbr_app_component
  PRIVATE include
)

target_link_libraries(lbr_app_component
  lbr_fri_ros2
)

ament_target_dependencies(lbr_app_component
  rclcpp
  rclcpp_components
  realtime_tools
  # fri
  lbr_fri_msgs
)

rclcpp_components_register_node(lbr_app_component
  PLUGIN lbr_fri_ros2::LBRApp
  EXECUTABLE lbr_app
)

install(
  TARGETS lbr_app_component
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

ament_package()
