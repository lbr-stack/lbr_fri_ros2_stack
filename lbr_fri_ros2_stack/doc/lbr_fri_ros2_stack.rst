LBR FRI ROS 2 Stack
===================
Collection of packages for controlling the KUKA LBR iiwa / med through ROS 2.

- ``lbr_bringup``: Launch package.
- ``lbr_demos``: Demo applications.
- ``lbr_description``: Description files.
- ``lbr_fri_msgs``: ``IDL``-equivalent of KUKA's ``nanopb`` message definitions.
- ``lbr_fri_ros2``: Exposes ``fri`` to ROS 2 topics / services.
- ``lbr_hardware_interface``: ``ros2_control`` hardware interface for the LBR.
- ``fri``: Integration of KUKA's Fast Research Interface (FRI) into ROS 2 ``ament_cmake`` build system.

Installation
------------
1. Install `colcon <https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html#install-colcon>`_, `rosdep <https://docs.ros.org/en/crystal/Installation/Linux-Install-Binary.html#installing-and-initializing-rosdep>`_ and `vcstool <https://github.com/dirk-thomas/vcstool#how-to-install-vcstool>`_.
2. Install the ``lbr_fri_ros2_stack``:

.. code-block:: bash

    mkdir -p lbr_fri_ros2_stack_ws/src && cd lbr_fri_ros2_stack_ws
    wget https://raw.githubusercontent.com/KCL-BMEIS/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos.yml -P src
    vcs import src < src/repos.yml
    rosdep install --from-paths src --ignore-src -r -y
    colcon build

Usage
-----
1. Launch the simulation / real robot:

.. code-block:: bash

    source install/setup.bash
    ros2 launch lbr_bringup lbr_bringup.launch.py model:=med7 sim:=true # model:=[iiwa7/iiwa14/med7/med14]

For the real robot, set ``sim:=false`` **and** follow :ref:`Robot Setup`.

2. Check-out demos, see :ref:`LBR Demos`.
