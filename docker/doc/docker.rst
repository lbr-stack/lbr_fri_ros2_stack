Docker
======
To run the ``lbr_fri_ros2_stack`` in a Docker container, follow the instructions below.

#. Install ``vcstool``

   .. code-block:: bash

    pip3 install vcstool

#. Create a workspace and clone

   .. code-block:: bash

    mkdir -p lbr-stack/src && cd lbr-stack
    vcs import src --input https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos.yaml

#. Copy the Dockerfile and the container scripts to the ``lbr-stack`` directory. Build and start the container

   .. code-block:: bash

    cp -r src/lbr_fri_ros2_stack/docker/* .
    chmod +x container_build.sh
    sudo ./container_build.sh 1.15 # replace by your FRI client version

#. Inside the container, launch e.g. simulation via (might require re-launch after Gazebo launched first time)

   .. code-block:: bash

    ros2 launch lbr_bringup bringup.launch.py model:=iiwa7 sim:=true moveit:=true

   .. note::

    List all arguments for the launch file via

    .. code-block:: bash

        ros2 launch lbr_bringup bringup.launch.py -s
