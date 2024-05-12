lbr_fri_ros2
============
.. note::
    
    Users will interface the hardware through :ref:`lbr_ros2_control`. Documentation for ``lbr_fri_ros2`` is intended for developers.

.. toctree::
   :caption: API Reference

   ../../../docs/doxygen/lbr_fri_ros2/html/annotated_classes

The ``lbr_fri_ros2`` package adds :lbr_fri_ros2:`AsyncClient <lbr_fri_ros2::AsyncClient>`.

Software Architecture
---------------------
An overview of the software architecture is shown :ref:`below <target to software architecture figure>`:

.. _target to software architecture figure:
.. thumbnail:: img/lbr_fri_ros2.drawio.svg
    :alt: lbr_fri_ros2

Design Principles
~~~~~~~~~~~~~~~~~
- Leave KUKA's FRI **untouched** (except for new ``ament_cmake`` build system) -> implemented through :ref:`FRI` package.
- Bridge ``nanopb`` (used within FRI for message definition) with ROS 2 Interface Definition Language (``IDL``) -> implemented through ``lbr_fri_idl`` package.
- Support future versions of the FRI -> implemented through ``vcstool`` and by separating the :ref:`FRI` package.
- Run stand-alone **and** within ``ros2_control`` -> implemented through :lbr_fri_ros2:`App <lbr_fri_ros2::App>`.

Implementation Details
~~~~~~~~~~~~~~~~~~~~~~
The FRI lets users communicate to the robot via a :fri:`ClientApplication <KUKA::FRI::ClientApplication>`. The :fri:`ClientApplication <KUKA::FRI::ClientApplication>` has (see :ref:`above <target to software architecture figure>`):

- :fri:`UdpConnection <KUKA::FRI::UdpConnection>` (UDP socket for reading states / sending commands)
- :fri:`Client <KUKA::FRI::LBRClient>` (interface for reading states / sending commands)

The user calls :fri:`step <KUKA::FRI::ClientApplication::step()>`, which, depending on the robot's state, callbacks:

- :fri:`monitor <KUKA::FRI::LBRClient::monitor()>`
- :fri:`waitForCommand <KUKA::FRI::LBRClient::waitForCommand()>`
- :fri:`command <KUKA::FRI::LBRClient::command()>`

The user can implement these callbacks to read states / send commands by implementing an :fri:`Client <KUKA::FRI::LBRClient>`.

The ``lbr_fri_ros2`` package implements an :fri:`Client <KUKA::FRI::LBRClient>` in :lbr_fri_ros2:`Client <lbr_fri_ros2::Client>`.

The :lbr_fri_ros2:`Client <lbr_fri_ros2::Client>` has

 - A publisher to publish states in :lbr_fri_ros2:`pub_lbr_state_ <lbr_fri_ros2::Client::pub_lbr_state_()>`.
 - A subscription to read commands in :lbr_fri_ros2:`on_lbr_command_ <lbr_fri_ros2::Client::on_lbr_command_(const lbr_fri_idl::msg::LBRCommand::SharedPtr lbr_command)>`.

Commands in :lbr_fri_ros2:`on_lbr_command_ <lbr_fri_ros2::Client::on_lbr_command_(const lbr_fri_idl::msg::LBRCommand::SharedPtr lbr_command)>` are checked for validity via a :lbr_fri_ros2:`CommandGuard <lbr_fri_ros2::CommandGuard>`.

API
~~~
For the ``Doxygen`` generated API, checkout `lbr_fri_ros2 <../../../docs/doxygen/lbr_fri_ros2/html/hierarchy.html>`_.
