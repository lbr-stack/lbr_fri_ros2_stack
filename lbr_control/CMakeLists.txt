cmake_minimum_required(VERSION 2.8.3)
project(lbr_control)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()

catkin_install_python(PROGRAMS python/cartesian_pose_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
