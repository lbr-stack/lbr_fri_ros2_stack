cmake_minimum_required(VERSION 3.16.3)
project(lbr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install
install(
  DIRECTORY config gazebo launch meshes ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.dsv")

ament_package()
