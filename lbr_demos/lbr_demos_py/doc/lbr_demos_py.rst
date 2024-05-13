lbr_demos_py
============
Collection of basic usage examples for the ``lbr_fri_ros2`` package through Python.

.. note::
    These demos are compatible with and closely follow KUKA's FRI example applications.

.. warning::
    Do always execute in ``T1`` mode first.

Joint Sine Overlay
------------------
#. .. dropdown:: Launch the ``LBRJointSineOverlay`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_joint_sine_overlay.png

#. Launch the `joint_sine_overlay.launch.py <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/launch/joint_sine_overlay.launch.py>`_ launch file:

    .. code-block:: bash

        ros2 launch lbr_demos_py joint_sine_overlay.launch.py model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

    The robot will move to the initial position via position control, then execute a rotation on joint ``A1``. A sinusoidal motion is overlayed on joint ``A4`` via `joint_sine_overlay_node <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/lbr_demos_py/joint_sine_overlay_node.py>`_.

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

Hardware
~~~~~~~~
#. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_lbr_server.png

#. Select

    - ``FRI send period``: ``10 ms``
    - ``IP address``: ``your configuration``
    - ``FRI control mode``: ``POSITION_CONTROL`` or ``JOINT_IMPEDANCE_CONTROL``
    - ``FRI client command mode``: ``POSITION``

#. Proceed with steps 1 and 2 from `Simulation`_ but with ``sim:=false``.

Torque Sine Overlay
-------------------
#. .. dropdown:: Launch the ``LBRTorqueSineOverlay`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_torque_sine_overlay.png

#. Launch the `torque_sine_overlay.launch.py <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/launch/torque_sine_overlay.launch.py>`_ launch file:

    .. code-block:: bash

        ros2 launch lbr_demos_py torque_sine_overlay.launch.py model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

    The robot will move to the initial position via joint impedance control. A sinusoidal torque is overlayed on joint ``A4`` via `torque_sine_overlay_node <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/lbr_demos_py/torque_sine_overlay_node.py>`_.

Wrench Sine Overlay
-------------------
#. .. dropdown:: Launch the ``LBRWrenchSineOverlay`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_wrench_sine_overlay.png

#. Launch the `wrench_sine_overlay.launch.py <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/launch/wrench_sine_overlay.launch.py>`_ launch file:

    .. code-block:: bash

        ros2 launch lbr_demos_py wrench_sine_overlay.launch.py model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

    The robot will move to the initial position via cartesian impedance control. A sinusoidal force is overlayed on the x- and y-axis via `wrench_sine_overlay_node <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/lbr_demos_py/wrench_sine_overlay_node.py>`_.
