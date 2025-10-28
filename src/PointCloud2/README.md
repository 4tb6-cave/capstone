# Point Cloud Extractor

This package extracts the point cloud data from the ROS bag message.

## Steps: 
1) Start ROS2 with ToF sensor: `ros2 run sipeed_tof_ms_a010 publisher --ros-args -p device:="/dev/ttyUSB0"`.
2) Verify topic list by: `ros2 topic list` and check if required topic is subscribed to. (Ex. "/cloud")
3) Run `colcon build`
... (in progress)
