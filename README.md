# Fast Robot Interface ROS2

This folder will contain a lightweight ROS wrapper for KUKA's Fast Robot Interface, for which we modified the build system to a target based CMake approach, to be cross platform compatible, as well as modular.

# Build

```shell
mkdir -p fri_ws/src
cd fri_ws/src
wget https://raw.githubusercontent.com/KCL-BMEIS/fri_ros2/master/fri_ros2.repos?token=AGJFDTREOXP5MD2A7D7BBVC6GBTCQ -O fri_ros2.repos
vcs import < fri_ros2.repos --workers=1 # limit workers due to private repos
```

```shell
source /opt/ros/dashing/setup.bash # or different ROS2 distribution
cd fri_ws
colcon build
```

# Launch
## Gazebo

```shell
source fri_ws/install/setup.bash
ros2 launch lbr_gazebo med7.launch.py
```

## Real Robot

```shell
source fri_ws/install/setup.bash
ros2 launch lbr_bringup lbr.launch.py
```

## Rviz2

```shell
source fri_ws/install/setup.bash
ros2 launch lbr_bringup lbr_rviz.launch.py
```

# Notes
iiwa stack https://github.com/IFL-CAMP/iiwa_stack
turtlebot3 https://github.com/ROBOTIS-GIT/turtlebot3
urdfexport https://github.com/syuntoku14/fusion2urdf
