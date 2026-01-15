FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
	python3-colcon-ros \
	libopencv-dev python3-opencv \
	ros-humble-cv-bridge

ENV ROS_WS=/ros2_ws
WORKDIR $ROS_WS

COPY ./sipeed_tof_ms_a010 ./src/sipeed_tof_ms_a010
COPY ./bno08x_ros2_driver ./src/bno08x_ros2_driver

RUN . /opt/ros/humble/setup.sh && \
	colcon build

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]