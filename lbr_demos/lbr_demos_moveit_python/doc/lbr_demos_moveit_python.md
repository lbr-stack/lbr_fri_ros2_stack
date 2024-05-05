# LBR Demos Moveit Python
1. Install Pilz industrial motion planner
```shell
sudo apt install ros-humble-pilz-industrial-motion-planner

```
2. Run simulation or real robot
```shell
ros2 launch lbr_bringup bringup.launch.py \
    moveit:=true \
    model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
```

3. Run sequenced motion example
```shell
ros2 run lbr_demos_moveit_python sequenced_motion
```
