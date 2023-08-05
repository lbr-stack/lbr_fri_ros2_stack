LBR Demos
=========
Demos for controlling the LBR through the Fast Robot Interface (FRI) from ROS 2.

.. warning::
    On the real robot, do always execute in ``T1`` mode first.

LBR Demos FRI ROS 2
-------------------
.. note::
    These demos are compatible with and closely follow KUKA's FRI example applications. They send commands to ``/lbr/command`` and read states from ``/lbr/state``.

.. note::
    A real robot is required to run these demos. Make sure you followed :ref:`Robot Setup` first.

.. toctree::
    :titlesonly:

    LBR Demos FRI ROS 2 C++ <../lbr_demos_fri_ros2_cpp/doc/lbr_demos_fri_ros2_cpp.rst>
    LBR Demos FRI ROS 2 Python <../lbr_demos_fri_ros2_python/doc/lbr_demos_fri_ros2_python.rst>
    LBR Demos FRI ROS 2 Advanced C++ <../lbr_demos_fri_ros2_advanced_cpp/doc/lbr_demos_fri_ros2_advanced_cpp.rst>
    LBR Demos FRI ROS 2 Advanced Python <../lbr_demos_fri_ros2_advanced_python/doc/lbr_demos_fri_ros2_advanced_python.rst>

LBR ROS 2 Control Demos
-----------------------
These demos demonstrate the LBR integration into the ROS 2 ecosystem through ``ros2_control``.

.. note::
    These demos run in simulation **and** on the real robot. For the real robot, make sure you followed :ref:`Robot Setup` first.

.. toctree::
    :titlesonly:

    LBR Demos ROS 2 Control C++ <../lbr_demos_ros2_control_cpp/doc/lbr_demos_ros2_control_cpp.rst>
    LBR Demos ROS 2 Control Python <../lbr_demos_ros2_control_python/doc/lbr_demos_ros2_control_python.rst>
