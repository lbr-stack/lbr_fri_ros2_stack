LBR FRI ROS 2
=============
The ``lbr_fri_ros2`` package provides a ROS 2 interface for the KUKA LBRs. It is designed to run stand-alone **and** within ``ros2_control``.

Quick Start
-----------
.. warning::
    Do always execute in ``T1`` mode first.

.. note::
    Make sure you followed the install instructions in :ref:`Robot Setup`.

1. .. dropdown:: Launch the ``LBRServer`` application on the ``KUKA smartPAD``

    .. thumbnail:: ../../lbr_demos/doc/img/applications_lbr_server.png

2. Run the :lbr_fri_ros2:`LBRApp <lbr_fri_ros2::LBRApp>` node via `lbr_app.launch.py <https://github.com/KCL-BMEIS/lbr_fri_ros2_stack/blob/galactic/lbr_fri_ros2/launch/lbr_app.launch.py>`_:

.. code-block:: bash

    ros2 launch lbr_fri_ros2 lbr_app.launch.py model:=iiwa7 # [iiwa7, iiwa14, med7, med14]

This does 2 things:

    - Loads the ``robot_description`` (for joint limits)
    - Runs the :lbr_fri_ros2:`LBRApp <lbr_fri_ros2::LBRApp>` node, which
        
        - Publishes robot states to ``/lbr_state``
        - Reads robot commands from ``/lbr_command``

See :ref:`LBR Demos` for more examples.

Software Architecture
---------------------
An overview of the software architecture is shown :ref:`below <target to software architecture figure>`:

.. _target to software architecture figure:
.. thumbnail:: img/lbr_fri_ros2.drawio.svg
    :alt: lbr_fri_ros2

Design Principles
~~~~~~~~~~~~~~~~~
- Leave KUKA's FRI **untouched** (except for new ``ament_cmake`` build system) -> implemented through :ref:`FRI` package.
- Bridge ``nanopb`` (used within FRI for message definition) with ROS 2 Interface Definition Language (``IDL``) -> implemented through ``lbr_fri_msgs`` package.
- Support future versions of the FRI -> implemented through ``vcstool`` and by separating the :ref:`FRI` package.
- Run stand-alone **and** within ``ros2_control`` -> implemented through :lbr_fri_ros2:`LBRApp <lbr_fri_ros2::LBRApp>`.

Implementation Details
~~~~~~~~~~~~~~~~~~~~~~
The FRI lets users communicate to the robot via a :fri:`ClientApplication <KUKA::FRI::ClientApplication>`. The :fri:`ClientApplication <KUKA::FRI::ClientApplication>` has (see :ref:`above <target to software architecture figure>`):

- :fri:`UdpConnection <KUKA::FRI::UdpConnection>` (UDP socket for reading states / sending commands)
- :fri:`LBRClient <KUKA::FRI::LBRClient>` (interface for reading states / sending commands)

The user calls :fri:`step <KUKA::FRI::ClientApplication::step()>`, which, depending on the robot's state, callbacks:

- :fri:`monitor <KUKA::FRI::LBRClient::monitor()>`
- :fri:`waitForCommand <KUKA::FRI::LBRClient::waitForCommand()>`
- :fri:`command <KUKA::FRI::LBRClient::command()>`

The user can implement these callbacks to read states / send commands by implementing an :fri:`LBRClient <KUKA::FRI::LBRClient>`.

The ``lbr_fri_ros2`` package implements an :fri:`LBRClient <KUKA::FRI::LBRClient>` in :lbr_fri_ros2:`LBRClient <lbr_fri_ros2::LBRClient>` that writes states to / reads commands from :lbr_fri_ros2:`LBRIntermediary <lbr_fri_ros2::LBRIntermediary>`.

The :lbr_fri_ros2:`LBRIntermediary <lbr_fri_ros2::LBRIntermediary>` is shared with :lbr_fri_ros2:`LBRApp <lbr_fri_ros2::LBRApp>`. :lbr_fri_ros2:`LBRApp <lbr_fri_ros2::LBRApp>` **exposes** the robot to ROS 2. It runs :lbr_fri_ros2:`step_ <lbr_fri_ros2::LBRApp::step_()>` in a thread, which does the following:

1. Reads commands from ``/lbr_command``.
2. Writes commands to :lbr_fri_ros2:`LBRIntermediary <lbr_fri_ros2::LBRIntermediary::command_to_buffer>` via :lbr_fri_ros2:`command_to_buffer <lbr_fri_ros2::LBRIntermediary::command_to_buffer(const lbr_fri_msgs::msg::LBRCommand::ConstSharedPtr)>`.
3. Calls :fri:`step <KUKA::FRI::ClientApplication::step()>`, which (through calling back :lbr_fri_ros2:`LBRClient <lbr_fri_ros2::LBRClient>`)

    - Reads commands from :lbr_fri_ros2:`LBRIntermediary <lbr_fri_ros2::LBRIntermediary::command_to_buffer>` via :lbr_fri_ros2:`buffer_to_command <lbr_fri_ros2::LBRIntermediary::buffer_to_command(KUKA::FRI::LBRCommand &) const>`.
    - Writes states to :lbr_fri_ros2:`LBRIntermediary <lbr_fri_ros2::LBRIntermediary::command_to_buffer>` via :lbr_fri_ros2:`state_to_buffer <lbr_fri_ros2::LBRIntermediary::state_to_buffer(const KUKA::FRI::LBRState &)>`.

4. Reads states from :lbr_fri_ros2:`LBRIntermediary <lbr_fri_ros2::LBRIntermediary>` via :lbr_fri_ros2:`LBRIntermediary::buffer_to_state <lbr_fri_ros2::LBRIntermediary::buffer_to_state()>`.
5. Publishes states to ``/lbr_state``.

The publishing of states and reading of commands is implemented via ``realtime_tools`` so that :lbr_fri_ros2:`step_ <lbr_fri_ros2::LBRApp::step_()>` is executed in a real-time-safe manner.

API
~~~
For the ``Doxygen`` generated API, checkout `lbr_fri_ros2 <../../../docs/doxygen/lbr_fri_ros2/html/hierarchy.html>`_.
