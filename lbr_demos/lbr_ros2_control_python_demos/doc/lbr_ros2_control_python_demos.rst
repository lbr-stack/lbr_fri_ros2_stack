LBR ROS 2 Control Python Demos
==============================
Collection of basic usage examples for the ``lbr_fri_hardware_interface`` through Python.

🙌 **Note**: These examples can be run in simulation **and** on the real robot.

⚠️ **General Safety**: Do always execute in simulation first, then in ``T1`` mode on the real robot.

Joint Trajectory Controller
---------------------------
Simulation
~~~~~~~~~~
1. Launch the ``LBRBringup``:

.. code-block:: bash

    ros2 launch lbr_bringup lbr_bringup.launch.py sim:=true model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

2. Run the `lbr_joint_trajectory_executioner_node <https://github.com/KCL-BMEIS/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_ros2_control_python_demos/lbr_ros2_control_python_demos/lbr_joint_trajectory_executioner_node.py>`_:

.. code-block:: bash

    ros2 run lbr_ros2_control_python_demos lbr_joint_trajectory_executioner_node

The robot will twist, then move to the zero configuration.

Real Robot
~~~~~~~~~~
1. Launch the ``LBRServer`` application on the ``KUKA smartPAD``.
2. Select

    - ``FRI send period``: ``10 ms``
    - ``IP address``: ``your configuration``
    - ``FRI control mode``: ``POSITION_CONTROL`` or ``JOINT_IMPEDANCE_CONTROL``
    - ``FRI client command mode``: ``POSITION``
3. Proceed with steps 1 and 2 from `Simulation`_ but with ``sim:=false``.
