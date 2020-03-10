cmake_minimum_required(VERSION 2.8.3)
project(lbr_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp moveit_ros_planning_interface geometry_msgs
)

catkin_package()

add_executable(endeffector_pose_node src/endeffector_pose_node.cpp)
target_include_directories(endeffector_pose_node
  PUBLIC
    ${catkin_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(endeffector_pose_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})