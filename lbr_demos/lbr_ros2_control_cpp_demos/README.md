# LBR ROS 2 Control C++ Demos
Collection of basic usage examples for the `lbr_fri_hardware_interface` through C++. **Note**: These examples can be run in simulation and on the real robot.

⚠️ **General Safety**: Do always execute in simulation first, then in `T1` mode on the real robot.

## Joint Trajectory Controller
### Simulation
1. Launch the `LBRBringup`:
```shell
ros2 launch lbr_bringup lbr_bringup.launch.py sim:=true model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
```

2. Run the [lbr_joint_trajectory_executioner_node](src/lbr_joint_trajectory_executioner_node.cpp):
```shell
ros2 run lbr_ros2_control_cpp_demos lbr_joint_trajectory_executioner_node
```
The robot will twist, then move to the zero configuration.

### Real Robot
1. Launch the `LBRServer` application on the `KUKA smartPAD`.
2. Select
    - `FRI send period`: `10 ms`
    - `IP address`: `your configuration`
    - `FRI control mode`: `POSITION_CONTROL` or `JOINT_IMPEDANCE_CONTROL`
    - `FRI client command mode`: `POSITION`
3. Proceed with steps 1 and 2 from [Simulation](#simulation) but with `sim:=false`.
