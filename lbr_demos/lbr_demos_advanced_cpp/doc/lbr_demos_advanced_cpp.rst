lbr_demos_advanced_cpp
======================
Collection of advanced usage examples for the ``lbr_fri_ros2`` package through C++.

.. warning::
    Do always execute in ``T1`` mode first.

Admittance Controller
---------------------
.. warning::
    Not well behaved around singularities, put the robot in a well-behaved configuration first, e.g. ``A1 = 0°``, ``A2 = -30°``, ``A3 = 0°``, ``A4 = 60°``, ``A5 = 0°``, ``A6 = -90°``, ``A7 = 0°``. This can be done using the ``smartPAD`` in ``T1`` mode.

#. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_lbr_server.png

#. Launch the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup bringup.launch.py \
            sim:=false \
            ctrl:=lbr_joint_position_command_controller \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Launch the `admittance_control <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_advanced_cpp/src/admittance_control_node.cpp>`_:octicon:`link-external`:

    .. code-block:: bash    
    
        ros2 run lbr_demos_advanced_cpp admittance_control --ros-args \
            -r __ns:=/lbr \
            --params-file `ros2 pkg prefix lbr_demos_advanced_cpp`/share/lbr_demos_advanced_cpp/config/admittance_control.yaml

#. Now gently move the robot at the end-effector.

Pose Controller
---------------
This demo uses ``KDL`` to calculate forward kinematics and inverse
kinematics to move the robot's end-effector along the z-axis in Cartesian space.

#. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_lbr_server.png

#. Launch the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup bringup.launch.py \
            sim:=false \
            ctrl:=lbr_joint_position_command_controller \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Launch the pose control

    .. code-block:: bash
    
        ros2 run lbr_demos_advanced_cpp pose_control --ros-args \
            -r __ns:=/lbr

#. Launch the path planning

    .. code-block:: bash
    
        ros2 run lbr_demos_advanced_cpp pose_planning --ros-args \
            -r __ns:=/lbr
