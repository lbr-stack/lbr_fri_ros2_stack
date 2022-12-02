cmake_minimum_required(VERSION 3.8)
project(lbr_fri_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lbr_fri_msgs REQUIRED)
find_package(fri REQUIRED)

add_library(lbr_fri_ros2
  SHARED 
    src/lbr_state_client.cpp
    src/lbr_app.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE include src
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  realtime_tools
  std_srvs
  lbr_fri_msgs
  fri
)

ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  realtime_tools
  lbr_fri_msgs
  fri
)

install(
  DIRECTORY include/
  DESTINATION include
)
  
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

# # test node
add_executable(lbr_app_node src/lbr_app_node.cpp)
target_include_directories(lbr_app_node PRIVATE include)
target_link_libraries(lbr_app_node ${PROJECT_NAME})
ament_target_dependencies(lbr_app_node
  rclcpp
  realtime_tools
  std_srvs
  lbr_fri_msgs
  fri
)

install(
  TARGETS lbr_app_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# test node

install(
    PROGRAMS test/lbr_command_node.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
