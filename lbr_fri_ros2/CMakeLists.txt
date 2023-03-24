cmake_minimum_required(VERSION 3.16)
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
    src/lbr_app_node.cpp
    src/lbr_client.cpp
    src/lbr_intermediary.cpp
    src/lbr_limit_guard.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE include
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
  std_srvs
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

add_executable(lbr_spinner src/lbr_spinner.cpp)
target_include_directories(lbr_spinner PRIVATE include)
target_link_libraries(lbr_spinner ${PROJECT_NAME})
ament_target_dependencies(lbr_spinner
  rclcpp
)

install(
  TARGETS lbr_spinner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
