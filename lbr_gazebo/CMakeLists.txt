cmake_minimum_required(VERSION 3.10.2)
project(lbr_gazebo)

# default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

# install
install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}/
)

# ament macros
ament_export_dependencies(gazebo_ros_pkgs)
ament_package()
