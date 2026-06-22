lbr_dual_arm
============
Dual-arm ``iiwa7`` integration demo, including: launch files, description file, and ``ros2_control`` configurations.

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Quick Start
-----------
Open a terminal, then run:

.. code-block:: bash

    ros2 launch lbr_dual_arm mock.launch.py \
        moveit:=true

.. note::
    For hardware, run ``hardware.launch.py`` (follow :doc:`Hardware Setup <../../../../lbr_fri_ros2_stack/doc/hardware_setup>` first).

.. note::
    To configure position / orientation, launch with ``lbr_one_x:=<value>`` etc. (list all arguments via ``ros2 launch lbr_dual_arm mock.launch.py -s``)

Description File
----------------
Custom description files can be generated via ``xacro``, see  `lbr_dual_arm.xacro <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/jazzy/lbr_demos/lbr_dual_arm/lbr_dual_arm/urdf/lbr_dual_arm.xacro>`_:octicon:`link-external`.

#. Include macros:

    .. code-block:: xml

        <xacro:include filename="$(find lbr_iiwa7_r800_description)/urdf/lbr_iiwa7_r800_macro.xacro" />
        <xacro:include filename="$(find lbr_ros2_control)/config/lbr_system_interface.xacro" />

#. Instantiate macros (``ros2_control`` plugin and robot description)

    .. code-block:: xml

        <xacro:lbr_system_interface ... />
        <xacro:lbr_iiwa7_r800 ... />

Done! The controller manager will load appropriate plugins when reading the robot description from the robot state publisher.

Customize Hardware Network Settings
-----------------------------------
#. Open `lbr_one_system_config.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/jazzy/lbr_demos/lbr_dual_arm/lbr_dual_arm/config/lbr_one_system_config.yaml>`_:octicon:`link-external` and `lbr_two_system_config.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/jazzy/lbr_demos/lbr_dual_arm/lbr_dual_arm/config/lbr_two_system_config.yaml>`_:octicon:`link-external`.
#. Set unique ``port_id`` values for each arm.
#. Set ``remote_host`` per arm according to your network setup.

.. note::
    ``port_id`` must be different for both arms.
