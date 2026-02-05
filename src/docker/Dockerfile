FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
	python3-colcon-ros \
	libopencv-dev python3-opencv \
	ros-humble-cv-bridge \
	libpcl-dev \
	ros-humble-pcl-conversions

ENV ROS_WS=/ros2_ws
WORKDIR $ROS_WS

COPY ./sipeed_tof_ms_a010 ./src
COPY ./PointCloud2 ./src

ARG BUILD_WORKSPACE=0
RUN if [ "$BUILD_WORKSPACE" = "1" ]; then \
		. /opt/ros/humble/setup.sh && colcon build; \
	fi

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
