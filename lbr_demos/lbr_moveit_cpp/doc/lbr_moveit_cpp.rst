lbr_moveit_cpp
==============
.. note::

    Also refer to the official `MoveIt <https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html>`_:octicon:`link-external` documentation.

MoveIt via C++ API
------------------
Simulation
~~~~~~~~~~
#. Run the robot driver:

    .. code-block:: bash

        ros2 launch lbr_bringup bringup.launch.py \
            moveit:=true \
            sim:=true \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

#. Run the `hello_moveit <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_moveit_cpp/src/hello_moveit.cpp>`_:octicon:`link-external` node:

    .. code-block:: bash

        ros2 launch lbr_moveit_cpp hello_moveit.launch.py \
            sim:=true \
            model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

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

#. Proceed with steps 1 and 2 from `Simulation`_ but with ``sim:=false``.
