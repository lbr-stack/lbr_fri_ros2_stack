cmake_minimum_required(VERSION 3.22)
project(lbr_dual_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
