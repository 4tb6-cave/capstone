# OakDriver class description
This class exists to facilitate recording of data with oak d pro using the depthai api.
# Public Methods
## \_\_init\_\_
Create driver for Oak D Pro camera (and maybe others).

Args:
	config: either a string representing the path to a yaml file with the desired configuration, or a dictionary containing the configuration
	baseDir: the path to the directory in which to store data. Created if it does not exist.

Returns:
	OakDriver object
## start_record
Start recording in a background process. Return True on success.
## stop_record
Stop recording that was started by start_record(). Return True on success.
## is_recording
Return True if recording is active.
## set_config_file
Set configuration from given yaml file.

Any parameters that are not specified will be set to their default values. This function cannot be called while recording.

Args:
	configFile: Path to a yaml file with desired configuration

Returns:
	True on successful saving of configuration
## set_config_dict
Set configuration from given dictionary.

Any parameters that are not specified will be set to their default values. This function cannot be called while recording.

Args:
	config: Dictionary with desired configuration

Returns:
	True on successful saving of configuration
## set_directory
Set directory in which recordings are made.

If the directory specified by baseDir parameter doesn't exist, it will be created.

Args:
	baseDir: directory in which to store recordings
## make_pipeline
Return depthai.Pipeline object created according to configuration.
## record
Record data from oak d pro
No parameters or return value
# Attributes
## baseDir
Directory in which to store data, set by set_directory or passed into constructor.
## config
Dictionary containing configuration. See [[#configuration]] for available parameters and their default values.
## dataDir
Path to data which was most recently recorded. A directory within baseDir with the name data_xxx. Every time start_record() is called and the recording process begins, this will be updated to a new directory.
## paths
Dictionary with paths to data.
Keys: rgb, left, right, depth, imu, time, config_save

These keys are not defined until \_define_paths() is called by the recording process, aka after start_record() is called.
# configuration
``` yaml
rgb:
  resolution: 1080p
  encoding: mjpeg
  lossless: true
  quality: 97
mono:
  resolution: 720p
  encoding: mjpeg
  lossless: true
  quality: 97
imu:
  frequency: 200
  batchThreshold: 10
  maxBatchReports: 20
stereo:
  medianFilterOn: false
  medianFilterKernel: 3x3
  extendedDisparity: true
  subpixel: false
  confidenceThreshold: 200
  leftRightCheck: 5
laserDotIntensity: 1
floodlightIntensity: 0
fps: 30
```
# Private functions and methods
see source code, if you aren't looking at source code you don't need to know anyways and they are all documented pretty well in there.
# to do
Add more depth post processing options
https://docs.luxonis.com/software/depthai/examples/depth_post_processing/

Add camera settings like exposure, shutter speed
