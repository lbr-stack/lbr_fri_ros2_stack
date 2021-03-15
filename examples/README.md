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
roslaunch lbr_moveit moveit_planning_execution.launch sim:=true # true by default
```
In another terminal (dont forget to source), launch the moveit example
```shell
roslaunch moveit_python_examples moveit_motion_examples.launch
```
The last command launches a ROS node via a [launch file](moveit_python_examples/launch/moveit_motion_examples.launch). The [ROS node](moveit_python_examples/src/moveit_motion_examples.py) then controls the robot via MoveIt!.

## C++
In this folder, we demonstrate simple motion view MoveIt! in C++.

### Pose Streaming Node
Execute end-effector poses as stored in a `csv` file. To Execute, do
```shell
cd fri_ros_ws
source devel/setup.bash # source this package
```
Launch the robot
```shell
roslaunch lbr_moveit moveit_planning_execution.launch sim:=true # true by default
```
In another terminal (dont forget to source), launch the moveit example
```shell
roslaunch moveit_cpp_examples stream_endeffector_pose_node.launch
```
This node reads in data from [data.csv](moveit_cpp_examples/data/data.csv) and executes it on the robot. The file to be read in can be changed by parsing a flag in the launch, e.g.
```shell
roslaunch moveit_cpp_examples stream_endeffector_pose_node.launch data:=<path/to/data.csv>
```
Note that the data in `data.csv` is expected to hold 4 columns, where the first one is the time, second, third and fourth are `x,y,z` position, respectively.
