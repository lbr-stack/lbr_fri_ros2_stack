lbr_demos_moveit_py
===================
.. warning::
    On hardware, do always execute in ``T1`` mode first.

MoveIt via RViz
-----------------
.. image:: img/iiwa7_moveit_rviz.png
    :align: center
    :alt: MoveIt via RViz
**IIWA 7 R800 in RViz**

To run MoveIt via RViz, simply follow:

Simulation
~~~~~~~~~~
#. Run the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup mock.launch.py \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run MoveIt with RViz:

    .. code-block:: bash

        ros2 launch lbr_bringup move_group.launch.py \
            mode:=mock \
            rviz:=true \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. You can now move the robot via MoveIt in RViz!

Hardware
~~~~~~~~
#. Client side configurations:

    #. Configure the ``client_command_mode`` to ``position`` in `lbr_system_parameters.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_ros2_control/config/lbr_system_parameters.yaml>`_:octicon:`link-external`
    #. Set the ``update_rate`` to ``100`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_ros2_control/config/lbr_controllers.yaml>`_:octicon:`link-external`

#. Remote side configurations:

    #. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

        .. thumbnail:: ../../doc/img/applications_lbr_server.png

    #. Select

        - ``FRI send period``: ``10 ms``
        - ``IP address``: ``your configuration``
        - ``FRI control mode``: ``POSITION_CONTROL`` or ``JOINT_IMPEDANCE_CONTROL``
        - ``FRI client command mode``: ``POSITION``

#. Proceed with steps 1 and 2 from `Simulation`_ but with ``ros2 launch lbr_bringup hardware.launch.py``.
