# Fast Robot Interface ROS
ROS packages for the KUKA LBR, including communication to the real robot via the Fast Robot Interface ([FRI](https://github.com/KCL-BMEIS/fri)), [MoveIt](https://moveit.ros.org/) integration and [Gazebo](http://gazebosim.org/) simulation support. To get going, follow the [First Steps](#first-steps).

# First Steps
The first steps allow you to use the robot, and to try out some [examples](#examples). Therefore, ensure the [prerequisites](#prerequisites) are fulfilled.

## Prerequisites
### Robot Operating System and Gazebo
Make sure the Robot Operating System [ROS](http://wiki.ros.org/melodic/Installation) is installed. For simulation support, install [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu).
### Synchronize Client Application (optional)

#### Sunrise Workbench (optional)
Install Sunrise Workbench on your computer. This step requires Windows as OS. Sunrise Workbench is KUKA's Java IDE that allows you to program the LBR. 
* Download it from the [RViM shared folder](https://emckclac.sharepoint.com/:u:/s/MT-BMEIS-RVIM/ETBf6gp3Ko5EvtJVziR8MZ4BLdeX8ysF13jTVmVreq0iZA?e=XJyagD) 
* Extract the .zip file and run the Sunrise Workbench Setup
* Follow the install instructions

### Build Fast Robot Interface ROS
Source ROS e.g. `source /opt/ros/melodic/setup.bash`
```shell
mkdir -p fri_ws/src
cd fri_ws/src
git clone --recursive https://github.com/KCL-BMEIS/fri_ros.git
cd ..
rosdep install --rosdistro melodic --ignore-src --from-paths src
catkin_make
```

## Examples

### Launch
The robot can be launched into via TODO add real/simulation here
```
source devel/setup.bash
roslaunch lbr_moveit moveit_planning_execution.launch
```

# Additional Resources
Additional resources can be found on the school's [Sharepoint](https://emckclac.sharepoint.com).
 - Hardware
    - [LBR Med Quickstart](https://emckclac.sharepoint.com/sites/MT-BMEIS-RVIM/Shared%20Documents/docs/inventory/kuka_lbr_med_7_R800/LBR_Med_Quick_Start_en.pdf)
    - [LBR Med 7 R800, LBR Med 14 R820, Insructions for Use](https://emckclac.sharepoint.com/sites/MT-BMEIS-RVIM/Shared%20Documents/docs/inventory/kuka_lbr_med_7_R800/GA_LBR_Med_en.pdf)
    - [KUKA Sunrise Cabinet Med, Instructions for Use](https://emckclac.sharepoint.com/sites/MT-BMEIS-RVIM/Shared%20Documents/docs/inventory/kuka_lbr_med_7_R800/GA_KUKA_Sunrise_Cabinet_Med_en.pdf)
 - Software
    - [KUKA Sunrise.OS Med 1.15, KUKA Sunrise.Workbench Med 1.15, Operating and Programming Instructions for System Integrators](https://emckclac.sharepoint.com/sites/MT-BMEIS-RVIM/Shared%20Documents/docs/inventory/kuka_lbr_med_7_R800/GA_KUKA_SunriseOS_Med_115_en.pdf)
    - [KUKA Sunrise.FRI 1.15](https://emckclac.sharepoint.com/sites/MT-BMEIS-RVIM/Shared%20Documents/docs/inventory/kuka_lbr_med_7_R800/KUKA_SunriseFRI_115_en.pdf)

# Notes
Note that this repository is based on the [IIWA Stack](https://github.com/IFL-CAMP/iiwa_stack), which offers similar functionality. This repository, however, adds real time communication support and the medical versions of the KUKA LBR.
