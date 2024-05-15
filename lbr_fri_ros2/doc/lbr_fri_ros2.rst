lbr_fri_ros2
============
.. note::
    
    Users will interface the hardware through :ref:`lbr_ros2_control`. Documentation for ``lbr_fri_ros2`` is **intended for developers**.

.. toctree::
   :caption: API Reference

   ../../../docs/doxygen/lbr_fri_ros2/html/annotated_classes

Software Architecture
---------------------
The ``lbr_fri_ros2`` package extends the FRI with:

#. :lbr_fri_ros2:`AsyncClient <lbr_fri_ros2::AsyncClient>` for asynchronous communication to the hardware
#. :lbr_fri_ros2:`App <lbr_fri_ros2::App>` for running the :lbr_fri_ros2:`AsyncClient <lbr_fri_ros2::AsyncClient>` asynchronously

The software architecture is shown :ref:`below <lbr_fri_ros2 software architecture figure>` (click to expand):

.. _lbr_fri_ros2 software architecture figure:
.. thumbnail:: img/lbr_fri_ros2_v2.0.0.svg
    :alt: lbr_fri_ros2

The :ref:`lbr_ros2_control` package can be considered a **User** in the above figure. It builds on top of the ``lbr_fri_ros2`` package to provide a ROS 2 interface to the hardware.
