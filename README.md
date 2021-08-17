# Fast Robot Interface ROS
ROS packages for the KUKA LBR, including communication to the real robot via the Fast Robot Interface ([FRI](https://github.com/KCL-BMEIS/fri)), [MoveIt](https://moveit.ros.org/) integration and [Gazebo](http://gazebosim.org/) simulation support. To get going, follow the [First Steps](#first-steps).

# First Steps
The first steps allow you to use the robot, and to try out some [examples](#examples). Therefore, ensure the [prerequisites](#prerequisites) are fulfilled.

## Prerequisites
### Robot Operating System and Gazebo
Make sure the Robot Operating System [ROS](http://wiki.ros.org/melodic/Installation) is installed. For simulation support, install [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu).
### Build Fast Robot Interface ROS
Source your ROS distribution ie in a terminal 
```shell
source /opt/ros/melodic/setup.bash # might differ
```
Then
```shell
mkdir -p fri_ros_ws/src && cd fri_ros_ws/src
git clone --recursive https://github.com/KCL-BMEIS/fri_ros.git
cd ..
rosdep install --rosdistro melodic --ignore-src --from-paths src
catkin_make
```
### Setup the Controller
The controller (Sunrise Cabinet) receives commands from the ROS machine via the FRI. Therefore, the server application has to be pushed onto the Sunrise Cabinet.
- Connect an ethernet cable to port X66 on the Sunrise Cabinet
- By default, the controller's IP address is `172.31.1.147`, set your IP to `172.31.1.148` and ping the KUKA Sunrise Cabinet
- [Install Sunrise Workbench](#####install-sunrise-workbench) (Windows required)
- [Synchronize the Server Application](#####synchronize-the-server-application)
#### Install Sunrise Workbench
This step requires Windows as OS. Sunrise Workbench is KUKA's Java IDE that allows you to program the LBR. 
* Download it from the [RViM shared folder](https://emckclac.sharepoint.com/:u:/s/MT-BMEIS-RVIM/ETBf6gp3Ko5EvtJVziR8MZ4BLdeX8ysF13jTVmVreq0iZA?e=XJyagD) 
* Extract the .zip file and run the Sunrise Workbench Setup
* Follow the install instructions
#### Synchronize the Server Application
To push the server application that handles the communication to the robot
 - Create a new project in Sunrise Workbench, File -> New -> Sunrise project
 - Copy the contents of [server](server) to the `src` folder inside the Sunrise project 
 - In the Software tab of the StationSetup.cat, tick the `Fast Robot Interface Extension` box
 - Install settings to the controller, in the Installation tab of the StationSetup.cat, press install
 - Synchronize the Sunrise project
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
