# dont forget to chmod +x container_build.sh 
xhost + 

docker rm lbr_stack_container

docker build -t lbr_stack_container .

docker run -it \
	--network host \
	--ipc host \
	--volume ./src:/home/ros2_ws/src \
	--volume /tmp/.X11-unix:/tmp/.X11-unix \
	--volume /dev/shm:/dev/shm \
	--volume /dev:/dev --privileged \
	--env DISPLAY \
	--env QT_X11_NO_MITSHM=1 \
	--name lbr_stack_container \
	lbr_stack_container
