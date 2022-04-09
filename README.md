# LBR FRI ROS Stack
ROS packages for the KUKA LBR, including communication to the real robot via the Fast Robot Interface ([FRI](https://github.com/KCL-BMEIS/fri)), [MoveIt](https://moveit.ros.org/) integration and [Gazebo](http://gazebosim.org/) simulation support. Included are the `iiwa7`, `iiwa14`, `med7`, and `med14`. To get going, follow the [First Steps](#first-steps).

# First Steps
Install [catkin](http://wiki.ros.org/catkin), [rosdep](http://wiki.ros.org/rosdep), and [vcstool](https://github.com/dirk-thomas/vcstool#how-to-install-vcstool). Build this repository
```shell
mkdir -p lbr_fri_ros_ws/src && cd lbr_fri_ros_ws
wget https://raw.githubusercontent.com/KCL-BMEIS/lbr_fri_ros2_stack/noetic/lbr_fri_ros_stack/repos.yml -P src
vcs import src < src/repos.yml
rosdep install --rosdistro noetic --ignore-src --from-paths src
catkin_make
```
Next, launch an example via
```shell
source devel/setup.bash
roslaunch lbr_moveit moveit_planning_execution.launch model:=med7 sim:=true # model:=[iiwa7/iiwa14/med7/med14]
```
For execution on the real robot, the steps in [Real Setup](#real-setup) are to be followed. Once the controller is set up, run the `LBRServer` app on the KUKA smartPAD. Once running, establish a connection via
```shell
source install/setup.bash
roslaunch lbr_moveit moveit_planning_execution.launch model:=med7 sim:=false # model:=[iiwa7/iiwa14/med7/med14]
```

# Real Setup
## Setup the Controller
The controller (Sunrise Cabinet) receives commands from the ROS machine via the FRI. Therefore, the server application has to be pushed onto the Sunrise Cabinet.
- Connect an ethernet cable to port X66 on the Sunrise Cabinet
- By default, the controller's IP address is `172.31.1.147`, set your IP to `172.31.1.148` and ping the KUKA Sunrise Cabinet
- [Install Sunrise Workbench](#install-sunrise-workbench) (Windows required)
- [Synchronize the Server Application](#synchronize-the-server-application)
### Install Sunrise Workbench
This step requires Windows as OS. Sunrise Workbench is KUKA's Java IDE that allows you to program the LBR. 
* Download it from the [RViM shared folder](https://emckclac.sharepoint.com/:u:/s/MT-BMEIS-RVIM/ETBf6gp3Ko5EvtJVziR8MZ4BLdeX8ysF13jTVmVreq0iZA?e=XJyagD) 
* Extract the .zip file and run the Sunrise Workbench Setup
* Follow the install instructions
### Synchronize the Server Application
To push the server application that handles the communication to the robot
 - Create a new project in Sunrise Workbench, File -> New -> Sunrise project
 - Copy the contents of [server](server) to the `src` folder inside the Sunrise project 
 - In the Software tab of the StationSetup.cat, tick the `Fast Robot Interface Extension` box
 - Install settings to the controller, in the Installation tab of the StationSetup.cat, press install
 - Synchronize the Sunrise project

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
