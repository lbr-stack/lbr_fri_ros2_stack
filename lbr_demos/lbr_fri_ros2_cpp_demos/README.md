# LBR FRI ROS2 C++ Demos
Collection of basic usage examples for the `lbr_fri_ros2` through C++. **Note**: These demos closely follow `KUKA`'s `FRI` example applications.

## Joint Sine Overlay
1. Launch the `LBRJointSineOverlay` application on the `KUKA smartPAD`.

2. Launch [joint_sine_overlay](launch/joint_sine_overlay.launch.py) launch file:
```shell
ros2 launch lbr_fri_ros2_cpp_demos joint_sine_overlay.launch.py model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
```
The robot will move to the initial position via position control, then execute a rotation on joint `A1 = lbr_joint_0`. A sinusoidal motion is overlayed on joint `A4 = lbr_joint_3` via [joint_sine_overlay_node.cpp](src/joint_sine_overlay_node.cpp).

## Torque Sine Overlay
1. Launch the `LBRTorqueSineOverlay` application on the `KUKA smartPAD`.

2. Launch [torque_sine_overlay](launch/torque_sine_overlay.launch.py) launch file:
```shell
ros2 launch lbr_fri_ros2_cpp_demos torque_sine_overlay.launch.py model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
```
The robot will move to the initial position via joint impedance control. A sinusoidal torque is overlayed on joint `A4 = lbr_joint_3` via [torque_sine_overlay_node.cpp](src/torque_sine_overlay_node.cpp).

## Wrench Sine Overlay
1. Launch the `LBRWrenchSineOverlay` application on the `KUKA smartPAD`.

2. Launch [wrench_sine_overlay](launch/wrench_sine_overlay.launch.py) launch file:
```shell
ros2 launch lbr_fri_ros2_cpp_demos wrench_sine_overlay.launch.py model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
```
The robot will move to the initial position via cartesian impedance control. A sinusoidal force is overlayed on the x- and y-axis via [wrench_sine_overlay_node.cpp](src/wrench_sine_overlay_node.cpp).
