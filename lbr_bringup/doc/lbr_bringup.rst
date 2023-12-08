LBR Bringup
===========
The ``lbr_fri_ros2_stack`` is designed for research **and** deployment. It runs standalone, with ``ros2_control``, and thus also with ``MoveIt 2``. Details are described in below sections

- :ref:`ROS 2 Control and MoveIt 2 Launch`
- :ref:`Standalone Launch`

Users may also refer to :ref:`Software Architecture` for a better understanding of the underlying ``lbr_fri_ros2`` package.

.. note::
    For the real robot, make sure you have followed :ref:`Robot Setup` first.

.. warning::
    On the real robot, do always execute in ``T1`` mode first.

General Information on the FRI
------------------------------
The ``FRI`` lets the user select a ``FRI control mode`` and a ``FRI client command mode``. When running the ``LBRServer``:

- .. dropdown:: Select ``FRI control mode``

    .. thumbnail:: ../../lbr_fri_ros2_stack/doc/img/controller/raw/lbr_server_control_mode.png

- .. dropdown:: Select ``FRI client command mode``
    
    .. thumbnail:: ../../lbr_fri_ros2_stack/doc/img/controller/raw/lbr_server_client_command_mode.png

The ``FRI control mode`` specifies the mode in which the robot is controlled, and the ``FRI client command mode`` specifies the commands that the user sends.

ROS 2 Control and MoveIt 2 Launch
---------------------------------
The ``lbr_bringup`` works for the simulation and the real robot. Run:

.. code:: bash

    ros2 launch lbr_bringup bringup.launch.py \
        model:=iiwa7 # [iiwa7, iiwa14, med7, med14] \
        sim:=true # [true, false] \
        rviz:=true # [true, false] \
        moveit:=true # [true, false]

.. note::
    For a list of available parameters, call ``ros2 launch lbr_bringup bringup.launch.py -s``.

When using the real robot

.. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../lbr_demos/doc/img/applications_lbr_server.png

and select:

- ``FRI send period``: ``10 ms``
- ``IP address``: ``your configuration``
- ``FRI control mode``: ``POSITION_CONTROL`` or ``JOINT_IMPEDANCE_CONTROL`` (will put the robot into a compliant mode)
- ``FRI client command mode``: ``POSITION``

Make sure that the ``update_rate`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble/lbr_ros2_control/config/lbr_controllers.yaml>`_ is greater or equal ``100`` (``FRI send period``).

For using other ``FRI send period``, also change the ``sample_time`` in the `lbr_system_interface.xacro <https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_ros2_control/config/lbr_system_interface.xacro>`_ (automated in the future).

Standalone Launch
-----------------
Standalone launch is great for research. Only the the real robot is supported. It can be launched through:

.. code:: bash

    ros2 launch lbr_fri_ros2 app.launch.py \
        model:=iiwa7 # [iiwa7, iiwa14, med7, med14] \
        robot_name:=lbr # any robot name

This runs the :lbr_fri_ros2:`AppComponent <lbr_fri_ros2::AppComponent>`, which creates 2 topics, ``/robot_name/command`` for commands and ``/robot_name/state``. See :ref:`LBR Demos FRI ROS 2` for more examples and :ref:`LBR FRI ROS 2` for more documentation.

.. note::
    For a list of available parameters, call ``ros2 launch lbr_fri_ros2 app.launch.py -s``.

Troubleshooting
---------------
Noisy Execution
~~~~~~~~~~~~~~~
Three main causes:

- Frequency: Make sure the ``ros2_control_node`` runs at the same or a higher rate of the ``FRI send period``, change ``update_rate`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble/lbr_ros2_control/config/lbr_controllers.yaml>`_. 
- Standalone noise: Smoothing might be required, see :ref:`LBR Demos FRI ROS 2`.
- Realtime priority: Set real time priority in ``code /etc/security/limits.conf``, add the line: ``user - rtprio 99``, where user is your username.
