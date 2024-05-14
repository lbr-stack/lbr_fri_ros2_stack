lbr_demos_cpp
=============
add table of contents TODO with sim / real matrix

.. warning::
    Do always execute in ``T1`` mode first.

Joint Sine Overlay
------------------
#. .. dropdown:: Launch the ``LBRJointSineOverlay`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_joint_sine_overlay.png

#. Select

    - ``FRI send period``: ``10 ms``
    - ``IP address``: ``your configuration``
    - ``FRI control mode``: ``POSITION_CONTROL``
    - ``FRI client command mode``: ``POSITION``

#. Run the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup bringup.launch.py \
            ctrl:=lbr_joint_position_command_controller \
            sim:=false \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]


    The robot will move to the initial position through the Java application.

#. Run the `joint_sine_overlay <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_cpp/src/joint_sine_overlay.cpp>`_:octicon:`link-external` node:

    .. code-block:: bash

        ros2 run lbr_demos_cpp joint_sine_overlay --ros-args -r __ns:=/lbr

    This node overlays a sinusoidal motion on joint ``A4``.

Joint Trajectory Controller
---------------------------
Simulation
~~~~~~~~~~
#. Launch the simulated robot:

    .. code-block:: bash

        ros2 launch lbr_bringup bringup.launch.py sim:=true model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `joint_trajectory_client <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_cpp/src/joint_trajectory_client.cpp>`_:octicon:`link-external`:

    .. code-block:: bash

        ros2 run lbr_demos_cpp joint_trajectory_client --ros-args -r __ns:=/lbr

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

#. Select

    - ``FRI send period``: ``2 ms``
    - ``IP address``: ``your configuration``
    - ``FRI control mode``: ``JOINT_IMPEDANCE_CONTROL``
    - ``FRI client command mode``: ``TORQUE``

#. Run the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup bringup.launch.py \
            ctrl:=lbr_torque_command_controller \
            sim:=false \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

    The robot will move to the initial position through the Java application.

#. Run the `torque_sine_overlay <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_cpp/src/torque_sine_overlay.cpp>`_:octicon:`link-external` node:

    .. code-block:: bash

        ros2 run lbr_demos_cpp torque_sine_overlay --ros-args -r __ns:=/lbr

    This node overlays a sinusoidal torque on joint ``A4``.

Wrench Sine Overlay
-------------------
#. .. dropdown:: Launch the ``LBRWrenchSineOverlay`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_wrench_sine_overlay.png

#. Select

    - ``FRI send period``: ``2 ms``
    - ``IP address``: ``your configuration``
    - ``FRI control mode``: ``CARTESIAN_IMPEDANCE_CONTROL``
    - ``FRI client command mode``: ``WRENCH``

#. Run the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup bringup.launch.py \
            ctrl:=lbr_wrench_command_controller \
            sim:=false \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

    The robot will move to the initial position through the Java application.

#. Run the `wrench_sine_overlay <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_cpp/src/wrench_sine_overlay>`_:octicon:`link-external` node:

    .. code-block:: bash

        ros2 run lbr_demos_cpp wrench_sine_overlay --ros-args -r __ns:=/lbr

    This node overlays a sinusoidal force on the x- and y-axis.
