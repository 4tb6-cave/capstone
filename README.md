# Project Name

Developer Names:

Date of project start:

This project is ...

The folders and files for this project are as follows:

docs - Documentation for the project
refs - Reference material used for the project, including papers
src - Source code
test - Test cases
etc.


## Recording data

Docker is used to simplify environment set up, especially because ROS is so finicky.

Use these commands to start and stop recording:

```sh
sudo docker compose -f src/record-compose.yml up
sudo docker compose -f src/record-compose.yml down
```

Data is saved as ROS bags in the `bag` directory.

## Processing

High level overview:

Point clouds and IMU data come from the rosbags created in recording.  
ICP is used to estimate the transformation between point cloud frames.  
GTSAM is used to fuse ICP transformation estimates with ICP (in progress) to find poses for each frame.  
The point clouds are added together and filtered to remove excess points.


Example of how I do this currently:
```sh
colcon build

DIR=
mkdir $DIRring6
BAG=rosbag2_2026_03_11-20_53_50
./build/point_cloud2/imu_extract $BAG $DIR

# make sure imu is sorted by message timestamp (ros bag may have received messages out of order)
cd $DIR
{ head -n 1 imu.csv; tail -n +2 imu.csv | sort -t, -k1,1g; } > tmp && mv tmp imu.csv
cd -

./build/point_cloud2/point_cloud_extract $BAG $DIR --topic /cloud_one --disable_sor

./src/scripts/downsample_timestamps.sh $DIR/time_stamps.csv $DIR/t4.csv 4
mv $DIR/temp $DIR/time_stamps.csv

./src/scripts/downsample_frames.sh $DIR/Filtered_Point_Clouds/ $DIR/f4 4
rm -r $DIR/Filtered_Point_Clouds
mv $DIR/f4 $DIR/Filtered_Point_Clouds

# inspect point clouds and choose starting and ending frame
f3d $DIR/Filtered_Point_Clouds

START=6
END=140
./build/point_cloud2/icp $DIR -s $START -e $END -g
./build/sensor_fusion/sensor_fusion $DIR --start $START --end $END
./build/point_cloud2/assemble_point_cloud $DIR -s $START -e $END --random_sample 0.2 --voxel_size 0.001

f3d $DIR/final_cloud.ply

```
