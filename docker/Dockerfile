FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

ARG FRI_CLIENT_VERSION=1.15

RUN apt update -y 

# create workspace
RUN mkdir -p home/ros2_ws/src
WORKDIR /home/ros2_ws/

COPY ./src /home/ros2_ws/src
RUN rosdep install -i --from-path src --rosdistro humble -y 

# "--symlink-install" allows the code in the locally mounted volume ./src to be adjusted without rebuilding
RUN colcon build --symlink-install --cmake-args -DFRI_CLIENT_VERSION=${FRI_CLIENT_VERSION} --no-warn-unused-cli

# source overlay workspace
RUN source /home/ros2_ws/install/local_setup.bash && echo "source /home/ros2_ws/install/local_setup.bash" >> ~/.bashrc
CMD ["/bin/bash"]
