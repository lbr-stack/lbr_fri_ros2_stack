cmake_minimum_required(VERSION 3.16.3)
project(lbr_state_broadcaster)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lbr_state_msgs REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
    src/${PROJECT_NAME}.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  controller_interface
  pluginlib
  lbr_state_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  
  ament_add_gmock(
    test_load_${PROJECT_NAME}
    test/test_load_${PROJECT_NAME}.cpp
  )
  target_include_directories(test_load_${PROJECT_NAME} PRIVATE include)
  ament_target_dependencies(test_load_${PROJECT_NAME}
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  controller_interface
  pluginlib
  lbr_state_msgs
)
  
ament_export_targets(
  ${PROJECT_NAME}Targets HAS_LIBRARY_TARGET
)

ament_package()
