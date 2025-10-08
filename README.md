# State of cave reconstruction system
## Recording
Data is recorded using OakDriver class (onto the pi or even directly to the laptop) which makes a folder.
Folder contains:
- config_save.yaml (configuration which was used when recording)
- imu.txt
- right.mjpeg
- left.mjpeg
- rgb.mjpeg
- depth (directory containing depth images saved as timestamp.png)
### viewing
Use ffmpeg to make an mp4 from mjpeg (or h265 or h264). use `ffmpeg -h` for more options and details, but for basic usage:
``` shell
ffmpeg -i rgb.mjpeg rgb.mp4
```
This puts video into a container that can be viewed by any video player.
## Offline processing
The processing is split up into several scripts, for ease of development. There is probably lots to optimize and streamline here, i just wrote this stuff quickly.
``` shell
# cd to directory containing data (or pass directory to scripts)

# 2024_brink_caves_summer location on this computer is ~/summer2024/cave_slam_2024

# Extract pngs from video stream and name with timestamps:
# use -h for details on arguments, or just run it in directory containing data with no arguments
python3 ~/summer2024/cave_slam_2024/offline_processing/prepare_video.py

# Rectify images for stereo processing:
# again, this can take a --source argument, just give it -h to find out details4
# This requires that calibration and stereo rectification parameters have been calculated and that maps for remap have been saved. Modify code (add argument?) to specify location.
python3 ~/summer2024/cave_slam_2024/offline_processing/rectify.py

# Construct depth from stereo with opencv
# use -h for details but you can view debug info and tune parameters using this script
python3 ~/summer2024/cave_slam_2024/offline_processing/opencv_stereo_depth.py .

```

### Open3D

Currently this is what has worked for me for easy sequences but it is far from the perfect solution. This is where the most work needs to be done.
Probably you want to make a new directory and copy over the depth_2 created by script above as well as left_rect. You can't use the rgb images until you figure out how to register them to the depth (they have a totally different FOV and everything) and I didn't get around to that. Maybe using opencv stereo calibration algorithms? For now I've just used left mono because that lines up exactly with depth images.
Rename directories to depth and image (that's what open3d wants)
Crop beginning and end if necessary, and delete images if there are too many.

Make config.json file

Example config.json with i think all of the parameters:
```
{
    "name": "cave data 10",
    "path_dataset": "/home/arco3/Desktop/aug_20/segment_3",
    "path_intrinsic": "/home/arco3/Desktop/aug_20/mono_rect_intrinsics.json",
    "n_frames_per_fragment": 50,
    "n_keyframes_per_n_frame": 5,
    "depth_min": 0.3,
    "depth_max": 3.0,
    "voxel_size": 0.05,
    "depth_diff_max": 0.07,
    "depth_scale": 1000,
    "preference_loop_closure_odometry": 0.1,
    "preference_loop_closure_registration": 5.0,
    "tsdf_cubic_size": 3.0,
    "icp_method": "generalized",
    "global_registration": "ransac",
    "python_multi_threading": true
}
```

``` shell
# cd to new directory

source ~/venv/open3d_icp/bin/activate

python ~/summer2024/cave_slam_2024/offline_processing/open3d_pipeline/src/reconstruction_system/run_system.py --config config.json --make --register --refine --integrate --debug
```
#### visualizing results
use --debug flag to see results as they are processed. Be ready with q key for when all the icp stuff comes up!

Or use these scripts to view after the fact.
```
python ~/summer2024/cave_slam_2024/offline_processing/open3d_pipeline/src/reconstruction_system/debug/visualize_fragments.py config.json

python ~/summer2024/cave_slam_2024/offline_processing/open3d_pipeline/src/reconstruction_system/debug/visualize_scene.py scene/integrated.ply
```
#### issues with open3d reconstruction pipeline
Running out of memory: it will often run out on the last part of the make step or the integrate step. To fix, use less pictures or increase voxel size and tsdf_cubic_size and you could also decrease depth_max but the default is sensible.
It doesn't have a good way to deal with bad data. I wish it was more robust and it could just say, this chunk is bad, so we will throw it out. instead that bad data makes everything else break too. This is especially bad in the caves.
## What's next
Try other reconstruction methods and slam algorithm.
RTAB-map looks promising, and Colmap may also work for this (feeding it the left and right and maybe even the rgb cameras, instead of just one).
Open3d could be made to work I think, but I think the reconstruction pipeline needs to be modified for the caves, it currently cannot perform reliably. It can do individual fragments pretty well when the data is friendly but as soon as it tries to put together good fragments and bad ones it fails terribly.

Try other sensors: lidar would be nice, other rgbd cameras.

Helmet hardware could use some improvement, although it functions pretty well. The camera sticks up to high (i hit it on a lot of rocks) and the user interface can't be used unless you take the helmet off.