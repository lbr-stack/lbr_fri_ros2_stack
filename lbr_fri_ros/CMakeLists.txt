cmake_minimum_required(VERSION 2.8.3)
project(lbr_fri_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  fri
  lbr_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp fri lbr_msgs hardware_interface controller_manager
)

add_executable(lbr_fri_ros_node src/lbr_fri_ros_node.cpp)

target_include_directories(lbr_fri_ros_node
  PUBLIC
    ${catkin_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(lbr_fri_ros_node
  ${catkin_LIBRARIES} ${fri_LIBRARIES}
)
