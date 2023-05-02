LBR FRI ROS 2
=============
The ``lbr_fri_ros2`` package provides a ROS 2 interface for the KUKA LBRs. It is meant to run stand-alone **and** within ``ros2_control``. For demonstrations see :doc:`LBR Demos <../../lbr_demos/doc/lbr_demos>`.

Software Architecture
---------------------
Design Principles
~~~~~~~~~~~~~~~~~
-   Leave KUKA's FRI **untouched**
-   Exchange FRI ``Makesfiles`` by ``CMake`` and in particular use ``ament_cmake`` ecosystem
-   Bridge ``nanopb`` (used within FRI for message definition) with ROS 2 Interface Definition Language (``IDL``)
-   Support future versions of the FRI
-   Run stand-alone **and** within ``ros2_control``

Implementation Details
~~~~~~~~~~~~~~~~~~~~~~

TODO:
- Inheritance diagram

- LBRApp (expose methods via services)
- LBRClient
- LBRIntermediary (shared)
- LBRCommandGuard (command check)

Full API
~~~~~~~~
``Doxygen`` generated documentation of the full API can be found here:

.. toctree::

    fri <../../../docs/doxygen/fri/html/hierarchy>
    lbr_fri_ros2 <../../../docs/doxygen/lbr_fri_ros2/html/hierarchy>
