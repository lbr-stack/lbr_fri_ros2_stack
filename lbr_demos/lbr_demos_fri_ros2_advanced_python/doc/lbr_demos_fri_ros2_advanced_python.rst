LBR Demos FRI ROS 2 Advanced Python
===================================
Collection of advanced usage examples for the ``lbr_ros2_control`` package through Python.

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

    ros2 launch lbr_bringup bringup.launch.py sim:=false ctrl:=forward_lbr_position_command_controller model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `admittance_control_node <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_fri_ros2_advanced_python/lbr_demos_fri_ros2_advanced_python/admittance_control_node.py>`_:

.. code-block:: bash

    ros2 run lbr_demos_fri_ros2_advanced_python admittance_control_node --ros-args -r __ns:=/lbr

#. Now gently move the robot at the end-effector.

Admittance Controller with Remote Center of Motion
--------------------------------------------------
#. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_lbr_server.png

#. Launch the robot driver:

.. code-block:: bash

    ros2 launch lbr_bringup bringup.launch.py sim:=false ctrl:=forward_lbr_position_command_controller model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `admittance_rcm_control_node <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_fri_ros2_advanced_python/lbr_demos_fri_ros2_advanced_python/admittance_rcm_control_node.py>`_:

.. code-block:: bash

    ros2 run lbr_demos_fri_ros2_advanced_python admittance_rcm_control_node --ros-args -r __ns:=/lbr

#. Now gently move the robot at the end-effector.
