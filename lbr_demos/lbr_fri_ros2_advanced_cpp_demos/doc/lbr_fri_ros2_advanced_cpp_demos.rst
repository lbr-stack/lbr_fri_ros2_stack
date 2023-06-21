LBR FRI ROS 2 Advanced C++ Demos
===================================
Collection of advanced usage examples for the ``lbr_fri_ros2`` package through C++.

.. warning::
    Do always execute in ``T1`` mode first.

Admittance Controller
---------------------
.. warning::
    Not well behaved around singularities, put the robot in a well-behaved configuration first, e.g. ``A1 = 0°``, ``A2 = -30°``, ``A3 = 0°``, ``A4 = 60°``, ``A5 = 0°``, ``A6 = -90°``, ``A7 = 0°``. This can be done using the ``smartPAD`` in ``T1`` mode.

1. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../doc/img/applications_lbr_server.png

2. Run the `admittance_control_node <https://github.com/KCL-BMEIS/lbr_fri_ros2_stack/blob/foxy/lbr_demos/lbr_fri_ros2_advanced_cpp_demos/src/admittance_control_node.cpp>`_:

.. code-block:: bash

    ros2 launch lbr_fri_ros2_advanced_cpp_demos admittance_control_node.launch.py model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

3. Now gently move the robot at the end-effector.
