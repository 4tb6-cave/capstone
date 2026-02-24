FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
	python3-colcon-ros \
	libopencv-dev python3-opencv \
	ros-humble-cv-bridge \
	python3-pip libgl1 \	
	libpcl-dev \
	ros-humble-pcl-conversions \
	libpcl-conversions-dev \
	libcli11-dev \
	nlohmann-json3-dev

RUN pip3 install numpy open3d --no-cache-dir


ENV ROS_WS=/ros2_ws
WORKDIR $ROS_WS

COPY ./sipeed_tof_ms_a010 ./src/sipeed_tof_ms_a010
#COPY ./bno08x_ros2_driver ./src/bno08x_ros2_driver
#COPY ./berk ./src/berk

#RUN . /opt/ros/humble/setup.sh && \
#    colcon build --packages-select berk

ARG BUILD_WORKSPACE=0
RUN if [ "$BUILD_WORKSPACE" = "1" ]; then \
		. /opt/ros/humble/setup.sh && colcon build; \
	fi

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
