LBR Demos ROS 2 Control Python
==============================
Collection of basic usage examples for ``lbr_ros2_control`` through Python.

.. note::
    These examples can be run in simulation **and** on the real robot.

.. warning::
    Do always execute in simulation first, then in ``T1`` mode on the real robot.

Joint Trajectory Controller
---------------------------
Simulation
~~~~~~~~~~
#. Launch the ``LBRBringup``:

.. code-block:: bash

    ros2 launch lbr_bringup bringup.launch.py sim:=true model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `joint_trajectory_executioner_node <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_ros2_control_python/lbr_demos_ros2_control_python/joint_trajectory_executioner_node.py>`_:

.. code-block:: bash

    ros2 run lbr_demos_ros2_control_python joint_trajectory_executioner_node

The robot will twist, then move to the zero configuration.

Real Robot
~~~~~~~~~~
#. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_lbr_server.png

#. Select

    - ``FRI send period``: ``10 ms``
    - ``IP address``: ``your configuration``
    - ``FRI control mode``: ``POSITION_CONTROL`` or ``JOINT_IMPEDANCE_CONTROL``
    - ``FRI client command mode``: ``POSITION``
#. Proceed with steps 1 and 2 from `Simulation`_ but with ``sim:=false``.
