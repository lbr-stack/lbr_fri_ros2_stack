LBR Demos
=========
Demos for controlling the LBR through the Fast Robot Interface (FRI) from ROS 2.

.. warning::
    On the real robot, do always execute in ``T1`` mode first.

LBR FRI ROS 2 Demos
-------------------
.. note::
    These demos are compatible with and closely follow KUKA's FRI example applications. They send commands to ``/lbr_command`` and read states from ``/lbr_state``.

.. note::
    A real robot is required to run these demos. Make sure you followed :ref:`Robot Setup` first.

.. toctree::
    :titlesonly:

    LBR FRI ROS 2 C++ Demos <../lbr_fri_ros2_cpp_demos/doc/lbr_fri_ros2_cpp_demos.rst>
    LBR FRI ROS 2 Python Demos <../lbr_fri_ros2_python_demos/doc/lbr_fri_ros2_python_demos.rst>
    LBR FRI ROS 2 Advanced C++ Demos <../lbr_fri_ros2_advanced_cpp_demos/doc/lbr_fri_ros2_advanced_cpp_demos.rst>
    LBR FRI ROS 2 Advanced Python Demos <../lbr_fri_ros2_advanced_python_demos/doc/lbr_fri_ros2_advanced_python_demos.rst>

LBR ROS 2 Control Demos
-----------------------
These demos demonstrate the LBR integration into the ROS 2 ecosystem through ``ros2_control``.

.. note::
    These demos run in simulation **and** on the real robot. For the real robot, make sure you followed :ref:`Robot Setup` first.

.. toctree::
    :titlesonly:

    LBR ROS 2 Control C++ Demos <../lbr_ros2_control_cpp_demos/doc/lbr_ros2_control_cpp_demos.rst>
    LBR ROS 2 Control Python Demos <../lbr_ros2_control_python_demos/doc/lbr_ros2_control_python_demos.rst>
