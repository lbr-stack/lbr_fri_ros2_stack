# LBR Examples
Simple examples for controlling the LBR robots through the Fast Robot Interface (FRI).

## Python
### Admittance Control
Demonstrates how to perform a hand-guiding application. Implementation in [scripts/admittance_control_node.py](scripts/admittance_control_node.py)
- Launch robot (works best if LBRServer is started with 10ms send period)
```shell
ros2 launch lbr_bringup lbr_bringup.launch.py model:=iiwa7 sim:=false
```
- Launch admittance control (make sure Python dependencies are installed)
```shell
ros2 run lbr_examples admittance_control_node.py
```

### LBR Command
Demonstrates how to read from and write to LBR robot by rotating the last joint. Implementation in [scripts/lbr_command_node.py](scripts/lbr_command_node.py)
- Launch robot
```shell
ros2 launch lbr_bringup lbr_bringup.launch.py model:=iiwa7 sim:=false
```
- Launch commanding node
```shell
ros2 run lbr_examples lbr_command_node.py
```

### Figure of Eight
Simple motion planning and control example. The end-effector follows a simple figure-of-eight pattern. Implementation in [scripts/figure_eight_move_to_start.py](scripts/figure_eight_move_to_start.py) and [scripts/figure_eight_node.py](scripts/figure_eight_node.py).
- Launch robot
```shell
ros2 launch lbr_bringup lbr_bringup.launch.py model:=med7 sim:=true controller:=forward_position_controller rviz_config:=fig8_config
```

- Move robot to start position
```shell
ros2 run lbr_examples figure_eight_move_to_start.py
```

- Run commanding node
```shell
ros2 run lbr_examples figure_eight_node.py
```

## C++
### LBR Command
Demonstrates how to read from and write to LBR robot by rotating the last joint. Implementation in [src/lbr_command_node.cpp](src/lbr_command_node.cpp)
- Launch robot
```shell
ros2 launch lbr_bringup lbr_bringup.launch.py model:=iiwa7 sim:=false
```
- Launch commanding node
```shell
ros2 run lbr_examples lbr_command_node
```
