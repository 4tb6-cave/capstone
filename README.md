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
sudo docker compose up -f src/record-compose.yml
sudo docker compose down -f src/record-compose.yml
```

Data is saved as ROS bags in the `bag` directory.

## Processing

High level overview:

Point clouds and IMU data come from the rosbags created in recording.  
ICP is used to estimate the transformation between point cloud frames.  
GTSAM is used to fuse ICP transformation estimates with ICP (in progress) to find poses for each frame.  
The point clouds are added together and filtered to remove excess points.