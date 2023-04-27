# LBR FRI ROS 2 Advanced Python Demos
Collection of advanced usage examples for the `lbr_fri_ros2` package.

## Admittance Controller
⚠️ **Warning**: Not well behaved around singularities, put robot in a well-behaved configuration first, e.g. `A1 = 0°`, `A2 = -30°`, `A3 = 0°`, `A4 = 60°`, `A5 = 0°`, `A6 = -90°`, `A7 = 0°`. Can be done using the `smartPAD` in `T1` mode.
1. Launch the `LBRServer` application on the `KUKA smartPAD`
2. Launch the robot driver
```shell
ros2 launch lbr_fri_ros2 lbr_app.launch.py model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
```
3. Run the admittance control node
```shell
ros2 run lbr_fri_ros2_advanced_python_demos admittance_control_node
```
4. Now gently move the robot at the end-effector.
