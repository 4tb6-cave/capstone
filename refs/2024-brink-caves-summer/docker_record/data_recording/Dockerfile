# parameterize ros distribution
ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO} as base

# install packages
RUN apt-get update && apt-get install -y