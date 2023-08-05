# lbr_fri_ros2_stack
![Build status](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build.yml/badge.svg?branch=humble) ![GitHub](https://img.shields.io/github/license/lbr-stack/lbr_fri_ros2_stack) 
[![Documentation Status](https://readthedocs.org/projects/lbr-fri-ros2-stack-doc/badge/?version=humble)](https://lbr-fri-ros2-stack-doc.readthedocs.io/en/humble/?badge=humble)

ROS 2 packages for the KUKA LBR, including communication to the real robot via the Fast Robot Interface ([FRI](https://github.com/lbr-stack/fri)) and [Gazebo](http://gazebosim.org/) simulation support. Included are the `iiwa7`, `iiwa14`, `med7`, and `med14`.

## Documentation
Full documentation available [here](https://lbr-fri-ros2-stack-doc.readthedocs.io/en/humble/index.html).

## Quick Start
Install [colcon](https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html#install-colcon), [rosdep](https://docs.ros.org/en/crystal/Installation/Linux-Install-Binary.html#installing-and-initializing-rosdep) and [vcstool](https://github.com/dirk-thomas/vcstool#how-to-install-vcstool). Build this repository
```shell
mkdir -p lbr_fri_ros2_stack_ws/src && cd lbr_fri_ros2_stack_ws
wget https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos.yml -P src
vcs import src < src/repos.yml
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```
Next, launch the simulation via
```shell
source install/setup.bash
ros2 launch lbr_bringup bringup.launch.py \
    model:=iiwa7 # [iiwa7, iiwa14, med7, med14] \
    sim:=true # [true, false] \
    rviz:=true # [true, false] \
    moveit:=true # [true, false]
```

Now, run the [demos](https://lbr-fri-ros2-stack-doc.readthedocs.io/en/humble/lbr_fri_ros2_stack/lbr_demos/doc/lbr_demos.html). To get started with the real robot, checkout the [Documentation](https://lbr-fri-ros2-stack-doc.readthedocs.io/en/humble/index.html) above.

## Acknowledgements
<img src="https://www.kcl.ac.uk/newimages/Wellcome-EPSRC-Centre-medical-engineering-logo.xa827df3f.JPG?f=webp" alt="wellcome" height="45" width="65" align="left">

This work was supported by core and project funding from the Wellcome/EPSRC [WT203148/Z/16/Z; NS/A000049/1; WT101957; NS/A000027/1]. 

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/b7/Flag_of_Europe.svg/1920px-Flag_of_Europe.svg.png" alt="eu_flag" height="45" width="65" align="left" >

This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 101016985 (FAROS project).
