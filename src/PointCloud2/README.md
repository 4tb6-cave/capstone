# Point Cloud Extractor

This package extracts the point cloud data from the ROS bag message.

## Steps: 
1) Start ROS2 with ToF sensor: `ros2 run sipeed_tof_ms_a010 publisher --ros-args -p device:="/dev/ttyUSB0"`.
2) Verify topic list by: `ros2 topic list` and check if required topic is subscribed to. (Ex. "/cloud")
3) Record ROS bag
4) Extract point clouds from ROS bag
5) Do ICP
6) Optimize poses for each frame using GTSAM
7) Put all of the frames together and filter


Todo:  
- Use IMU measurements (add to GTSAM somehow)
- Find ICP covariance to understand error better and improve optimization results
- Improve documentation
- Improve project structure
- Add visualization for different parts of algorithm to easily check if it is working
- Automatic loop closure (for now it is totally up to the human)


## Building

``` sh
colcon build
```


## point_cloud_extract
`point_cloud_extract` (point_cloud2_extract.cpp) reads a ROS bag and saves all of the TOF sensor data as .ply files

## icp
The `icp` executable (see icp_point_cloud.cpp) calculates the transformation between different point clouds.

Example:
``` sh
./icp Filtered_Point_Clouds --start 20 --end 50 --loop_closure_pair 20 50 --enable_gicp
```

## GTSAM smoothing
At this point the executable from the sensor_fusion directory is used to find the optimal pose for each frame.
This will read all transformations from the transform path (created by `icp`) including any loop closures.

``` sh
cd sensor_fusion
mkdir build
cd build
cmake ..
make

./sensor_fusion --transform_path ../../PointCloud2/transforms --start 20 --end 50
```

## assemble_point_cloud
`assemble_point_cloud` uses poses (stored as 4x4 transformation matrices in .csv files) pre-calculated for each TOF frame
and puts all of the point clouds together into one point cloud. Because this would result in an unmanageable number of
points, there are various filters included to reduce this. See the help message for the parameters for the filters.

``` sh
./assemble_point_cloud Filtered_Point_Clouds ../sensor_fusion/poses --start 20 --end 50 --random_sample 0.1
```