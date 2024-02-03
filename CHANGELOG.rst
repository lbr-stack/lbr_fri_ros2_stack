^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package LBR FRI ROS 2 Stack
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Humble v1.4.3 (2024-02-03)
--------------------------
* Fixes planning scene namespace https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/153
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/155

Humble v1.4.2 (2023-12-29)
--------------------------
* Fixes cartesian path for move group node with namespace: https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/133
* Solution @josefinemonnet: https://github.com/ros-planning/moveit2/issues/2545#issuecomment-1868480168
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/146

Humble v1.4.1 (2023-12-15)
--------------------------
* Removes the ``base_frame`` parameter from ``lbr_bringup``, ``lbr_description``, ``lbr_fri_ros2``, ``lbr_ros2_control``
* Updates RViZ default config in ``lbr_moveit_config``
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/144

Humble v1.4.0 (2023-12-08)
--------------------------
* The general goal of this release is a tighter ``ros2_control`` integration. The ``lbr_bringup``
  will serve as single entry point in the future. For now, ``app_component`` and ``app.launch.py`` are kept
* Changes to ``lbr_fri_ros2``:

  * Removes logging / parameter interfaces from ``lbr_fri_ros2`` (so ``lbr_ros2_control`` serves as single interaction point)
  * Updates legacy ``app_component`` in ``lbr_fri_ros2`` for changes. To be depracted in the future
  * Adds force-torque estimator to ``lbr_fri_ros2``
* Changes to ``lbr_ros2_control``:

  * Removes now redundant node from ``lbr_ros2_control``
  * Adds forward position and forward torque controllers to ``lbr_ros2_control``
  * Removes estimated force-torque broadcaster from ``lbr_ros2_control`` in favor of ``ros2_control`` default implementation
    
    * Force-torque now available under ``/lbr/force_torque_broadcaster/wrench`` 
    * Namespace issues since ``lbr_controllers.yaml`` includes namespace in ``frame_id`` parameter
  * Adds ``lbr_fri_ros2`` force-torque estimator to ``lbr_ros2_control`` as sensor
  * Adds configurations to ``lbr_system_interface.xacro``
  * Simplifies ``lbr_ros2_control`` class names
* ``/lbr/command/position`` topic now under ``/lbr/command/joint_position``
* Adds this changelog with release notes
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/142

Humble v1.3.1 (2023-11-21)
--------------------------
* v1.3.0 Gazebo namespace fixes in https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/123
* Fix iiwa ee link in https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/126
* Humble v.1.3.1 in https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/130
* Full log: https://github.com/lbr-stack/lbr_fri_ros2_stack/compare/humble-v1.3.0-beta...humble-v1.3.1

Humble v1.3.0 beta (2023-10-03)
-------------------------------
* Namespaced robot_description and joint_states
* De-coupled commands, user will interact through LBRPositionCommand, LBRTorqueCommand, LBRWrenchCommand
* Multi-robot support
* New command / state interfaces in lbr_fri_ros2
* Topic free ros2_control support through command / state interfaces in lbr_fri_ros2
* Intraprocess cpp admittance demo
* New app component based on command / state interfaces in lbr_fri_ros2
* Refers to https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/121

Humble v1.2.5 (2023-09-08)
--------------------------
* Updated visualization (STL -> DAE files with materials, might occur dark in Gazebo, caused by lack of light)
* Fixes joint bug in Gazebo
* Improved logging in command guard

Humble v1.2.4 (2023-08-09)
--------------------------
* Remove robot name from configs and use frame_prefix from robot state publisher instead
* Removed robot name from joint names, e.g. lbr_A1 -> A1
* Added PID for asynchronous control rate
* Simplified class names, e.g. LBRApp -> App
* Add utils.hpp for PID and exponential filter

Humble v1.2.3 (2023-08-07)
--------------------------
* Utilizes FRI through vendor package for common fri source in https://github.com/lbr-stack/
* Addresses some of https://github.com/lbr-stack/lbr_fri_ros2_stack/pull/85
* Give command guard only logger interface
* Fix open loop bug
* Adds real-time priority via rt_prio parameter

Humble v1.2.2 (2023-08-05)
--------------------------
* Adds base frame parameter to URDF and launch
* Adds an open loop option to control the robot, which works extremely well
* Updates logo in readme
* Updates joint names to KUKA convention, i.e. A1,...

Humble v1.2.1 (2023-08-04)
--------------------------
* Stack's new home at: https://github.com/lbr-stack

Humble v1.2.0 (2023-08-03)
--------------------------
* Re-introduces MoveIt, refer to https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/52
* Moves demo prefix to front for improved package overview
* Single node for hardware interface
* Static executors where possible
* Adds plenty documentation
* Introduce /lbr, i.e. robot name, namespace to LBRClient for better multi-robot support. Commands / states now e.g. published to /lbr/command / /lbr/state
* Hardware interface exact limits (stand-alone use has safety-limits)
* Gives command guard a node handle
