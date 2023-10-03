LBR FRI ROS 2 Stack
===================
Collection of packages for controlling the KUKA LBR iiwa / med through ROS 2.

- ``lbr_bringup``: Launch package.
- ``lbr_demos``: Demo applications.
- ``lbr_description``: Description files.
- ``lbr_fri_msgs``: ``IDL``-equivalent of KUKA's ``nanopb`` message definitions.
- ``lbr_fri_ros2``: Exposes ``fri`` to ROS 2 topics / services.
- ``lbr_ros2_control``: ``ros2_control`` integration for the LBR.
- ``lbr_moveit_config```: ``MoveIt 2`` configurations for thr LBR.
- ``fri``: Integration of KUKA's Fast Robot Interface (FRI) into ROS 2 ``ament_cmake`` build system.

Installation
------------
#. Install `colcon <https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html#install-colcon>`_, `rosdep <https://docs.ros.org/en/crystal/Installation/Linux-Install-Binary.html#installing-and-initializing-rosdep>`_ and `vcstool <https://github.com/dirk-thomas/vcstool#how-to-install-vcstool>`_.
#. Install the ``lbr_fri_ros2_stack``:

.. code-block:: bash

    mkdir -p lbr-stack/src && cd lbr-stack
    wget https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos.yaml -P src
    vcs import src < src/repos.yaml
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install

.. note::
    For the real robot, additional steps are required:

    .. toctree::

        robot_setup

Usage
-----
#. Launch the simulation / real robot:

.. code-block:: bash

    source install/setup.bash
    ros2 launch lbr_bringup bringup.launch.py model:=med7 sim:=true # model:=[iiwa7/iiwa14/med7/med14]

For the real robot, set ``sim:=false`` and run the ``LBRServer`` on the ``KUKA smartPAD``, see :ref:`Install Applications to the Robot`.

#. Checkout bringup and demos, see :ref:`LBR Bringup` and :ref:`LBR Demos`.
