LBR MoveIt Config
=================
Documentation for generating (:ref:`Generate MoveIt Configuration`) and updating (:ref:`Update MoveIt Configuration`) the ``MoveIt 2`` configurations for the LBRs. For full documentation see `MoveIt Documentation <https://moveit.picknik.ai/main/index.html>`_.

Generate MoveIt Configuration 
-----------------------------
This procedure applies to all LBRs: ``iiwa7``, ``iiwa14``, ``med7``, and ``med14``.

1. Make sure ``MoveIt 2`` is installed.

.. code-block:: bash

    sudo apt install ros-$ROS_DISTRO-moveit*

2. Make sure the ``lbr_fri_ros2_stack`` is installed and **sourced**, see :ref:`Installation`.

3. Launch the setup assistant

.. code-block:: bash

    ros2 launch moveit_setup_assistant setup_assistant.launch.py

4. .. dropdown:: Select the target ``xacro``, e.g. ``lbr_fri_ros2_stack_ws/install/lbr_description/share/lbr_description/urdf/iiwa7/iiwa7.urdf.xacro``, and ``Load Files``

    .. thumbnail:: img/moveit_setup_assistant.png

5. Configure to your needs. Some configurations we used:

- ``Robot Poses``: Add ``zero`` and ``transport``
- ``MoveIt Controller``: ``Auto Add FollowJointsTrajectory``
- ``Perception``: select ``None``
- ``Launch``: Unselect all but ``MoveGroup Launch``, ``RViz Launch and Config``, and ``Setup Assistant Launch``
- ``Configuration Files``: ``browse`` output folder and ``Generate Package``

6. Some manual changes are required:

has_acceleration_limits: true
max_acceleration: 0 to float

arm_controller -> position_trajectory_controller

generate_demo_launch: 

        {"use_sim_time": True}
        # additional_env={"DISPLAY": ":0"},


- Since ``URDF`` doesn't support acceleration limits, manually add acceleration limits in ``iiwa7_moveit_config/config/joint_limits.yaml``
- In the ``move_group.launch.py`` use the robot descriotion from ``lbr_description``
- In ``iiwa7_moveit_config/config/moveit_controllers.yaml`` change the ``arm_controller`` to ``position_trajectory_controller`` as in ``lbr_bringup/config/lbr_controllers.yml`` 

Update MoveIt Configuration 
---------------------------
1. Make sure the ``lbr_fri_ros2_stack`` is installed and sourced, see :ref:`Installation`.

2. Run the setup assistant for the existing configuration.

.. code-block:: bash

    ros2 launch iiwa7_moveit_config  setup_assistant.launch.py # [iiwa7, iiwa14, med7, med14]

3. Update and save the configurations, similar to :ref:`Generate MoveIt Configuration`.
