LBR Demos
=========
Demos for controlling the LBR through the Fast Robot Interface (FRI) from ROS 2.

⚠️ **General Safety**: Do always execute in *T1* mode on the real robot first.

LBR FRI ROS 2 Demos
-------------------
🤝 **Note**: These demos are compatible with and closely follow KUKA's FRI example applications. They send commands to */lbr_command* and read states from */lbr_state*.

🫣 **Note**: A real robot is required to run these demos.

.. toctree::
    :maxdepth: 1

    lbr_fri_ros2_python_demos/README
    lbr_fri_ros2_advanced_python_demos/README
    lbr_fri_ros2_cpp_demos/README

LBR ROS 2 Control Demos
-----------------------
These demos demonstrate the LBR integration into the ROS 2 ecosystem through *ros2_control*.

🙌 **Note**: These demos run in simulation **and** on the real robot.

.. toctree::
    :maxdepth: 1

    lbr_fri_ros2_python_demos/README
    lbr_fri_ros2_cpp_demos/README
