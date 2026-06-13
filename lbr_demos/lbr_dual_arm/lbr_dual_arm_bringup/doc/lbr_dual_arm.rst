lbr_dual_arm_bringup
====================
Launch package for the dual-arm ``iiwa7`` demo on ``humble``.

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Mock Bringup
------------
Terminal 1:

.. code-block:: bash

    ros2 launch lbr_dual_arm_bringup mock.launch.py

Terminal 2:

.. code-block:: bash

    ros2 launch lbr_dual_arm_bringup move_group.launch.py \
        rviz:=true

Hardware Bringup
----------------
Terminal 1:

.. code-block:: bash

    ros2 launch lbr_dual_arm_bringup hardware.launch.py

Terminal 2:

.. code-block:: bash

    ros2 launch lbr_dual_arm_bringup move_group.launch.py \
        mode:=hardware \
        rviz:=true
