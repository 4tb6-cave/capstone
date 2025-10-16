#!/usr/bin/env python3

import depthai as dai
import os
import cv2
import numpy as np
from tabulate import tabulate
import yaml
import subprocess
import multiprocessing

import time

class OakDriver:
    """
    Provide an easy to use and fully configurable interface to record with Depth AI cameras.

    Attributes:
        baseDir:
            Directory in which to store data, set by set_directory or passed into constructor.
        config:
            Dictionary containing configuration.
        dataDir:
            Path to data which was most recently recorded. A directory within baseDir with
            the name data_xxx. Every time start_record() is called and the recording process
            begins, this will be updated to a new directory.
        paths:
            Dictionary with paths to data, which contains these keys: rgb, left, right, depth, 
            imu, time, config_save. The values are meaningless until start_record() is called.

    Constants:
        RECORD_START: 0
        RECORD_OK: 1
        RECORD_STOP: 2
        RECORD_OFF: 3
        RECORD_ERROR: 4
    """

    RECORD_START = 0
    RECORD_OK = 1
    RECORD_STOP = 2
    RECORD_OFF = 3
    RECORD_ERROR = 4

    def __init__(self, config: dict | str | None = None, baseDir: str | None = None):
        """
        Create driver for Oak D Pro camera (and maybe others).
        
        Args:
            config: either a string representing the path to a yaml file with the
                desired configuration, or a dictionary containing the configuration
            baseDir: the path to the directory in which to store data. Created if it
                does not exist.

        Returns:
            OakDriver object
        """
        self._context = multiprocessing.get_context("spawn")
        self._recordStart = self._context.Event()
        self._recordStart.clear()
        self._recordProcess = self._context.Process(target=self._record)
        self._recordOkay = self._context.Event()
        self._recordOkay.clear()

        self.paths = {}

        if type(config) == dict:
            self.set_config_dict(config)
        elif type(config) == str:
            self.set_config_file(config)
        else:
            self.config = self._default_config
        
        if baseDir:
            self.set_directory(baseDir)
        else:
            self.baseDir = os.getcwd()

    def start_record(self):
        """
        Start recording in a background process. Return True on success.
        """
        if self._recordStart.is_set():
            print("Already recording!")
            return False
        
        if self._recordProcess.is_alive():
            print("Record process is already running although it shouldn't be!")
            return False
        
        # make data directory and define file paths
        self._define_paths()
        
        self._recordStart.set() # record flag set to true
        self._recordProcess.start() # start recording
        print("Started recording process")
        return True

    def stop_record(self):
        """
        Stop recording that was started by start_record(). Return True on success.
        """
        if not self._recordStart.is_set():
            print("Recording already stopped")
            return False
        
        success = True
        
        if not self._recordProcess.is_alive():
            print("Record process already died although it should be running")
            success = False
            # continue to clear flag and make new process so error can be recovered from

        self._recordStart.clear()
        self._recordProcess.join() # hangs until process ends.
        code = self._recordProcess.exitcode

        # must make a new process because old one cannot be restarted once stopped
        self._recordProcess = self._context.Process(target=self._record)

        if (code != 0):
            print("Record process exited with code", code)
            return False
        
        print("Ended recording process")
        return success
    
    def is_recording(self):
        """
        Return True if recording is active.
        """
        return self.get_recording_status() == self.RECORD_OK
    
    def get_recording_status(self):
        """
        Return status of recording.
        
        Value is either RECORD_OFF, RECORD_START, RECORD_OK, RECORD_STOP, or RECORD_ERROR
        """
        if not self._recordProcess.is_alive() and not self._recordOkay.is_set() and not self._recordStart.is_set():
            return self.RECORD_OFF
        
        if self._recordProcess.is_alive() and not self._recordOkay.is_set() and self._recordStart.is_set():
            return self.RECORD_START
        
        if self._recordProcess.is_alive() and self._recordOkay.is_set() and self._recordStart.is_set():
            return self.RECORD_OK

        if self._recordProcess.is_alive() and not self._recordOkay.is_set() and not self._recordStart.is_set():
            return self.RECORD_STOP
        
        else:
            return self.RECORD_ERROR
        
        # 0 0 0 off
        # 1 0 1 start
        # 1 1 1 ok
        # 1 0 0 stop

        # 0 0 1 might be true for a split second inbetween lines in start_record,
        #           otherwise likely to occur with error in depth ai or file opening
        # 0 1 0 very strange and probably serious error which should never occur
        # 0 1 1 same thing, it shouldn't happen because of the finally clause
        # 1 1 0 this too. as long as no other method tries to change

    def set_config_file(self, configFile: str):
        """
        Set configuration from given yaml file.

        Any parameters that are not specified will be set to their default values.
        This function cannot be called while recording.

        Args:
            configFile: Path to a yaml file with desired configuration

        Returns:
            True on successful saving of configuration
        """
        
        with open(configFile, 'r') as file:
            config = yaml.safe_load(file)
            return self.set_config_dict(config)

        
    def set_config_dict(self, config: dict):
        """
        Set configuration from given dictionary.

        Any parameters that are not specified will be set to their default values.
        This function cannot be called while recording.

        Args:
            config: Dictionary with desired configuration

        Returns:
            True on successful saving of configuration
        """
        # ensure recording is not already started
        if self._recordStart.is_set() or self._recordProcess.is_alive():
            print("Can't change configuration while recording is running")
            return False
        
        self.config = self._set_default(config, self._default_config)
        return True
    
    def set_directory(self, baseDir: str):
        """
        Set directory in which recordings are made.
        
        If the directory specified by baseDir parameter doesn't exist, it will be created.

        Args:
            baseDir: directory in which to store recordings
        """
        os.makedirs(baseDir, exist_ok=True)
        self.baseDir = baseDir
    
    def make_pipeline(self):
        """
        Return depthai.Pipeline object created according to configuration.
        """
            
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        camRGB = pipeline.create(dai.node.ColorCamera)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        encoderRGB = pipeline.create(dai.node.VideoEncoder)
        encoderLeft = pipeline.create(dai.node.VideoEncoder)
        encoderRight = pipeline.create(dai.node.VideoEncoder)
        stereo = pipeline.create(dai.node.StereoDepth)
        imu = pipeline.create(dai.node.IMU)
        outRGB = pipeline.create(dai.node.XLinkOut)
        outLeft = pipeline.create(dai.node.XLinkOut)
        outRight = pipeline.create(dai.node.XLinkOut)
        outDepth = pipeline.create(dai.node.XLinkOut)
        outIMU = pipeline.create(dai.node.XLinkOut)

        outRGB.setStreamName('outRGB')
        outLeft.setStreamName('outLeft')
        outRight.setStreamName('outRight')
        outDepth.setStreamName('outDepth')
        outIMU.setStreamName("imu")

        # Camera settings
        camRGB.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRGB.setResolution(self._get_resolution("rgb"))
        monoLeft.setCamera("left")
        monoLeft.setResolution(self._get_resolution("mono"))
        monoRight.setCamera("right")
        monoRight.setResolution(self._get_resolution("mono"))
        camRGB.setFps(self.config["fps"])
        monoLeft.setFps(self.config["fps"])
        monoRight.setFps(self.config["fps"])

        # IMU settings
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, self.config["imu"]["frequency"])
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, self.config["imu"]["frequency"])
        imu.setBatchReportThreshold(self.config["imu"]["batchThreshold"])
        imu.setMaxBatchReports(self.config["imu"]["maxBatchReports"])

        # stereo settings
        stereo.initialConfig.setMedianFilter(self._get_median_filter_settings())
        stereo.setExtendedDisparity(self.config["stereo"]["extendedDisparity"])
        stereo.setSubpixel(self.config["stereo"]["subpixel"])
        stereo.initialConfig.setConfidenceThreshold(self.config["stereo"]["confidenceThreshold"])
        stereo.setLeftRightCheck(self.config["stereo"]["leftRightCheck"])

        # encoder settings
        encoderRGB.setDefaultProfilePreset(self.config["fps"], self._get_encoding("rgb"))
        encoderLeft.setDefaultProfilePreset(self.config["fps"], self._get_encoding("mono"))
        encoderRight.setDefaultProfilePreset(self.config["fps"], self._get_encoding("mono"))
        
        if self.config["rgb"]["lossless"]:
            encoderRGB.setLossless(True)
        else:
            encoderRGB.setQuality(self.config["rgb"]["quality"])

        if self.config["mono"]["lossless"]:
            encoderLeft.setLossless(True)
            encoderRight.setLossless(True)
        else:
            encoderLeft.setQuality(self.config["mono"]["quality"])
            encoderRight.setQuality(self.config["mono"]["quality"])

        # Linking
        camRGB.video.link(encoderRGB.input)
        monoLeft.out.link(encoderLeft.input)
        monoRight.out.link(encoderRight.input)
        encoderRGB.bitstream.link(outRGB.input)
        encoderLeft.bitstream.link(outLeft.input)
        encoderRight.bitstream.link(outRight.input)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        stereo.depth.link(outDepth.input)
        imu.out.link(outIMU.input)

        return pipeline
        
    def _set_default(self, config, default):
        """
        Put missing parameters into config dictionary from default dictionary.

        Recursive function to check each level of the config dictionary and copy
        any missing key value pairs from default config. No checking of keys that 
        are present in config is performed.

        Args:
            config: dictionary to check for completeness
            default: dictionary to compare config to and to take default values from

        Returns:
            dictionary that consists of the key value pairs in config, plus
            any keys with their default values that exist in default but not in
            config.
        """

        # ignore anything that isn't a dictionary
        if not isinstance(default, dict):
            # print("end")
            return config
        
        # if default is a dict, config has to be too
        if not isinstance(config, dict):
            # print("config bad, reverting to default")
            return default
        
        # if both are dicts, iterate through all keys of default    
        for key in default:
            if key in config: # key exists, but it may refer to a dict
                config[key] = self._set_default(config[key], default[key])
            else: # key does not exist so default is used
                config[key] = default[key]
        
        return config
    
    def _define_paths(self):
        """
        Populate path variables. 
        
        Creates a new directory for data, called data_xxx, under directory passed into constructor
        or current working directory if not specified.
        """
        # create data directory
        i = 0
        while os.path.exists(os.path.join(self.baseDir, f'data_{i:03}')):
            i += 1
        self.dataDir = os.path.join(self.baseDir, f'data_{i:03}')
        os.makedirs(self.dataDir)

        # define paths
        self.paths["rgb"] = os.path.join(self.dataDir, "rgb." + self.config["rgb"]["encoding"])
        self.paths["left"] = os.path.join(self.dataDir, "left." + self.config["mono"]["encoding"])
        self.paths["right"] = os.path.join(self.dataDir, "right." + self.config["mono"]["encoding"])
        self.paths["depth"] = os.path.join(self.dataDir, "depth")
        os.mkdir(self.paths["depth"])
        self.paths["imu"] = os.path.join(self.dataDir, "imu.txt")
        self.paths["time"] = os.path.join(self.dataDir, 'time.txt')
        self.paths["config_save"] = os.path.join(self.dataDir,"config_save.yaml")

    def _record(self):
        """
        Record data from oak d pro.

        Called by start_record() in a seperate process, to run in the background
        """

        timesRGB = []
        timesLeft = []
        timesRight = []
        timesDepth = []

        pipeline = self.make_pipeline()

        # Connect to device and start pipeline
        with dai.Device(pipeline) as dev:

            # dev.setLogLevel(dai.LogLevel.DEBUG)
            # dev.setLogOutputLevel(dai.LogLevel.DEBUG)
            
            dev.setIrLaserDotProjectorIntensity(self.config["laserDotIntensity"])
            dev.setIrFloodLightIntensity(self.config["floodlightIntensity"])

            # Output queues will be used to get the encoded data from the outputs defined above
            qRGB = dev.getOutputQueue(name='outRGB', maxSize=30, blocking=True)
            qLeft = dev.getOutputQueue(name='outLeft', maxSize=30, blocking=True)
            qRight = dev.getOutputQueue(name='outRight', maxSize=30, blocking=True)
            qDepth = dev.getOutputQueue(name='outDepth', maxSize=30, blocking=True)
            qIMU = dev.getOutputQueue(name="imu", maxSize=50, blocking=False)

            with open(self.paths["rgb"], 'wb') as fileRGB, open(self.paths["left"], 'wb') as fileLeft, open(self.paths["right"], 'wb') as fileRight, open(self.paths["imu"], "w") as fileIMU:
                print("Recording data...")
                self._recordOkay.set()
                try:
                    while self._recordStart.is_set():
                        # Empty each queue
                        self._write_video(qRGB, fileRGB, timesRGB)
                        self._write_video(qLeft, fileLeft, timesLeft)
                        self._write_video(qRight, fileRight, timesRight)
                        self._write_still(qDepth, self.paths["depth"], timesDepth)
                        self._write_imu(qIMU, fileIMU)
                finally:
                    self._recordOkay.clear()

        # Make table
        maxLength = max(len(timesRGB), len(timesLeft), len(timesRight), len(timesDepth))
        col1 = np.array(timesRGB)
        col2 = np.array(timesLeft)
        col3 = np.array(timesRight)
        col4 = np.array(timesDepth)
        col1.resize(maxLength)
        col2.resize(maxLength)
        col3.resize(maxLength)
        col4.resize(maxLength)
        table = np.column_stack((col1, col2, col3, col4))
        table_string = tabulate(table, headers=["Color", "Left", "Right", "Depth"], floatfmt=".6f")
        
        # save timestamp table
        with open(self.paths["time"], 'w') as file:
            file.write(table_string)
        
        # save configuration
        with open(self.paths["config_save"], "w") as file:
            file.write(yaml.safe_dump(self.config))

    def make_mp4(self):
        for stream in ["rgb", "left", "right"]:
            subprocess.check_call(
                [
                    "ffmpeg",
                    "-framerate", 
                    "30",
                    "-i",
                    self.paths[stream],
                    "-c",
                    "copy",
                    os.path.join(self.dataDir, stream + ".mp4")
                ]
            )

    def make_stills(self):
        for stream in ["rgb", "left", "right"]:
            dest = os.path.join(self.dataDir, stream)
            os.mkdir(dest)
            subprocess.check_call(
                [
                    "ffmpeg",
                    "-i",
                    self.paths[stream],
                    os.path.join(dest, "%04d.png")
                ]
            )

    def _save_time(self, msg: dai.ImgFrame, l: list):
        """
        Extract timestamp and save it to list l.

        If an image has not been saved previously, there will be a blank space in the list
        """
        num = msg.getSequenceNum()
        while(len(l) < num + 1):
            l.append(0)
        l[num] = msg.getTimestampDevice().total_seconds()

    def _write_video(self, queue: dai.DataOutputQueue, video_file: str, times: list):
        """
        Save encoded video to file.

        Nonblocking, should be in while True loop or similar
        """
        # get next message from queue
        if queue.has():
            msg = queue.get()
            msg.getData().tofile(video_file)
            self._save_time(msg, times)
    
    def _write_still(self, queue: dai.DataOutputQueue, path: str, times: list):
        """
        Save picture to file in folder given by path.

        Nonblocking, should be in while True loop or similar
        """
        # get next message from queue
        if queue.has():
            msg = queue.get()
            filename = f'{msg.getTimestampDevice().total_seconds():.6f}.png'
            cv2.imwrite(os.path.join(path, filename), msg.getFrame())
            self._save_time(msg, times)

    def _write_imu(self, queue: dai.DataOutputQueue, imuFile: str):
        """
        Save imu data to file.

        Nonblocking, should be in while True loop or similar
        """
        if queue.has():
            msg = queue.get()
            for packet in msg.packets:
                a = packet.acceleroMeter
                g = packet.gyroscope
                timestamp = a.getTimestampDevice().total_seconds()
                imuFile.write(f"{timestamp:.6f} {g.x:.8f} {g.y:.8f} {g.z:.8f} {a.x:.8f} {a.y:.8f} {a.z:.8f}\n")

    def _get_resolution(self, camera):
        """
        Return object corresponding to resolution of camera
        """
        if camera == "rgb":
            match self.config["rgb"]["resolution"].lower():
                case "1080p":
                    return dai.ColorCameraProperties.SensorResolution.THE_1080_P
                case "4k":
                    return dai.ColorCameraProperties.SensorResolution.THE_4_K
                case "12mp":
                    return dai.ColorCameraProperties.SensorResolution.THE_12_MP
                case _:
                    return dai.ColorCameraProperties.SensorResolution.THE_1080_P

        if camera == "mono":
            match self.config["mono"]["resolution"].lower():
                case "800p":
                    return dai.MonoCameraProperties.SensorResolution.THE_800_P
                case "720p":
                    return dai.MonoCameraProperties.SensorResolution.THE_720_P
                case "400p":
                    return dai.MonoCameraProperties.SensorResolution.THE_400_P
                case _:
                    return dai.MonoCameraProperties.SensorResolution.THE_720_P
    

    def _get_median_filter_settings(self):
        """
        Return median filter config
        """
        if not self.config["stereo"]["medianFilterOn"]:
            return dai.MedianFilter.MEDIAN_OFF
        
        match self.config["stereo"]["medianFilterKernel"]:
            case "3x3":
                return dai.MedianFilter.KERNEL_3x3
            case "5x5":
                return dai.MedianFilter.KERNEL_5x5
            case "7x7":
                return dai.MedianFilter.KERNEL_7x7
            case _:
                return dai.MedianFilter.MEDIAN_OFF
    
    def _get_encoding(self, camera):
        """
        Return encoding configuration for camera (either mono or color)
        """
        if camera != "rgb" and camera != "mono":
            return dai.VideoEncoderProperties.Profile.MJPEG
        match self.config[camera]["encoding"].lower():
            case "h265":
                return dai.VideoEncoderProperties.Profile.H265_MAIN
            case "h265_main":
                return dai.VideoEncoderProperties.Profile.H265_MAIN
            case "h264":
                return dai.VideoEncoderProperties.Profile.H264_MAIN
            case "h264_main":
                return dai.VideoEncoderProperties.Profile.H264_MAIN
            case "h264_baseline":
                return dai.VideoEncoderProperties.Profile.H264_BASELINE
            case "h264_high":
                return dai.VideoEncoderProperties.Profile.H264_HIGH
            case "mjpeg":
                return dai.VideoEncoderProperties.Profile.MJPEG
            case _:
                return dai.VideoEncoderProperties.Profile.MJPEG
    
    _default_config = {
        "rgb":
            {"resolution": "1080p",
            "encoding": "mjpeg",
            "lossless": False,
            "quality": 97},
        "mono":
            {"resolution": "720p",
            "encoding": "mjpeg",
            "lossless": False,
            "quality": 97},
        "imu":
            {"frequency": 200,
            "batchThreshold": 10,
            "maxBatchReports": 20},
        "stereo":
            {"medianFilterOn": False,
            "medianFilterKernel": "3x3",
            "extendedDisparity": True,
            "subpixel": False,
            "confidenceThreshold": 200,
            "leftRightCheck": 5},
        "laserDotIntensity": 1,
        "floodlightIntensity": 0,
        "fps": 30
    }

if __name__=="__main__":
    d = OakDriver()
    d.start_record()
    while not (d.is_recording()):
        time.sleep(.5)

    time.sleep(10)
    d.stop_record()