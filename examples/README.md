# Examples
Collection of examples to control the lbr robot via [MoveIt!](https://moveit.ros.org/).
## Python
In this folder, we demonstrate simple motions via MoveIt!, for reference, check the MoveIt! [documentation](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html). Make sure this package is sourced, always do
```shell
cd fri_ros_ws
source devel/setup.bash # source this package
```
Launch the robot
```shell
roslaunch lbr_moveit moveit_planning_execution.launch sim:=false # false by default
```
In another terminal, launch the moveit example
```shell
roslaunch moveit_python_examples moveit_motion_examples.launch
```
The last command launches a ROS node via a [launch file](moveit_python_examples/launch/moveit_motion_examples.launch). The [ROS node](moveit_python_examples/src/moveit_motion_examples.py) then controls the robot via MoveIt!.
