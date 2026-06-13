lbr_dual_arm_description
========================

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Customize Robot Placement
-------------------------
#. Open ``urdf/lbr_dual_arm.xacro``.
#. Adjust the fixed joint origins:

    - ``lbr_one_base_joint``
    - ``lbr_two_base_joint``

#. Typical changes are:

    - Increase/decrease spacing between both arms via ``xyz``.
    - Rotate one arm around the base frame via ``rpy``.

Customize Hardware Network Settings
-----------------------------------
#. Open ``ros2_control/lbr_one_system_config.yaml`` and ``ros2_control/lbr_two_system_config.yaml``.
#. Set unique ``port_id`` values for each arm.
#. Set ``remote_host`` per arm according to your network setup.

.. note::
    ``port_id`` must be different for both arms.
