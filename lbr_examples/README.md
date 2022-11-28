# LBR Examples
Simple examples for controlling the LBR robots through the Fast Robot Interface (FRI).

## Python
### Sinusoidal
Demonstrates how to read from and write to LBR robot by rotating the last joint. Implementation in [scripts/lbr_sinusoidal_node.py](scripts/lbr_sinusoidal_node.py)
- Launch robot
```shell
ros2 launch lbr_bringup lbr_bringup.launch.py model:=iiwa7 sim:=true controller:=forward_position_controller
```
- Launch sinusoidal node
```shell
ros2 launch lbr_examples lbr_sinusoidal_node.launch.py
```
