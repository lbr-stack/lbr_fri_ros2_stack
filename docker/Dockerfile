FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

RUN apt update -y 
RUN apt install vim python-is-python3 python3-pip python3-colcon-common-extensions -y
RUN pip3 install scipy numpy
# create workspace and build package
# source workspace
RUN source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# create workspace
RUN mkdir -p home/ros2_ws/src
WORKDIR /home/ros2_ws/

COPY ./src /home/ros2_ws/src
RUN rosdep install -i --from-path src --rosdistro humble -y 

# "--symlink-install" allows the code in the locally mounted volume ./src to be adjusted without rebuilding
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# source overlay workspace
RUN source /home/ros2_ws/install/setup.bash && echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc
CMD ["/bin/bash"]


