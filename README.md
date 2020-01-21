# Fast Robot Interface ROS2

This folder will contain a lightweight ROS wrapper for KUKA's Fast Robot Interface, for which we modified the build system to a target based CMake approach, to be cross platform compatible, as well as modular.

# Build

```
mkdir -p fri_ws/src
cd fri_ws/src
wget https://raw.githubusercontent.com/KCL-BMEIS/fri_ros2/master/fri_ros2.repos # works once public
vcs import < fri_ros2.repos --workers=1 # limit workers due to private repos
```

# Notes
iiwa stack https://github.com/IFL-CAMP/iiwa_stack
turtlebot3 https://github.com/ROBOTIS-GIT/turtlebot3
urdfexport https://github.com/syuntoku14/fusion2urdf
