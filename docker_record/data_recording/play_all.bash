#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
for FILE in data/*; do
	ros2 bag info $FILE
	ros2 bag play $FILE
done
