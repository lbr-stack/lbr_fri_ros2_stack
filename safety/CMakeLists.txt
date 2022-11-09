cmake_minimum_required(VERSION 3.5)
project(safety)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)


# TEMP TEMP TEMP

add_executable(fake_joint_state src/test_publisher_node.cpp)
ament_target_dependencies(fake_joint_state rclcpp sensor_msgs)

install(TARGETS
  fake_joint_state
  DESTINATION lib/${PROJECT_NAME})

# TEMP TEMP TEMP

add_executable(safety_node src/safety_node.cpp)
target_include_directories(safety_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(safety_node rclcpp sensor_msgs)
ament_target_dependencies(safety_node Eigen3)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3 orocos_kdl kdl_parser)
target_link_libraries(safety_node ${kdl_parser_LIBRARIES})
target_link_libraries(safety_node
  ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(
  TARGETS safety_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
