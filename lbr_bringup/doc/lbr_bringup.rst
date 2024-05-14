lbr_bringup
===========
The ``lbr_bringup`` works for the simulation and the real robot. Run:

.. code:: bash

    ros2 launch lbr_bringup bringup.launch.py \
        model:=iiwa7 # [iiwa7, iiwa14, med7, med14] \
        sim:=true # [true, false] \
        rviz:=true # [true, false] \
        moveit:=true # [true, false]

.. note::
    For a list of available parameters, call ``ros2 launch lbr_bringup bringup.launch.py -s``.

When using the real robot, select ``sim:=false`` and

.. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../lbr_demos/doc/img/applications_lbr_server.png

Select

- ``FRI send period``: ``10 ms``
- ``IP address``: ``your configuration``
- ``FRI control mode``: ``POSITION_CONTROL`` or ``JOINT_IMPEDANCE_CONTROL`` (will put the robot into a compliant mode)
- ``FRI client command mode``: ``POSITION``

Users may also refer to :ref:`Software Architecture` for a better understanding of the underlying ``lbr_fri_ros2`` package.

.. note::
    For the real robot, make sure you have followed :doc:`Hardware Setup <../../lbr_fri_ros2_stack/doc/hardware_setup>` first.

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

Troubleshooting
---------------
Noisy Execution
~~~~~~~~~~~~~~~
- Frequency: Make sure the ``ros2_control_node`` frequency and the ``FRI send period`` are compatible, consider changing ``update_rate`` in `lbr_controllers.yaml <https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble/lbr_ros2_control/config/lbr_controllers.yaml>`_:octicon:`link-external`. 
- Realtime priority: Set real time priority in ``code /etc/security/limits.conf``, add the line: ``user - rtprio 99``, where user is your username.
