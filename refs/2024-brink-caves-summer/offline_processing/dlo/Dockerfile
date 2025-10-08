FROM osrf/ros:noetic-desktop-full

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# install packages
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-catkin-tools \
    libomp-dev \
    libpcl-dev \
    libeigen3-dev
    
RUN source /opt/ros/noetic/setup.bash \
    && mkdir ws && cd ws && mkdir src && catkin init && cd src \
    && git clone https://github.com/vectr-ucla/direct_lidar_odometry.git\
    && catkin build