# Fast Robot Interface ROS

This folder will contain a lightweight ROS wrapper for KUKA's Fast Robot Interface, for which we modified the build system to a target based CMake approach, to be cross platform compatible, as well as modular.

# Build

```shell
source /opt/ros/melodic/setup.bash
mkdir -p fri_ws/src
cd fri_ws/src
git clone --recursive --branch melodic https://github.com/KCL-BMEIS/fri_ros2.git
cd ..
rosdep install --rosdistro melodic --ignore-src --from-paths src
catkin_make
```

# Launch
The robot can be launched into via
```
source devel/setup.bash
roslaunch lbr_moveit moveit_planning_execution.launch
```

