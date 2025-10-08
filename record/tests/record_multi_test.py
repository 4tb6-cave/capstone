#!/usr/bin/env python3

import depthai as dai
from os import path
import cv2
import numpy as np

from multiprocessing import Process, Queue
import subprocess

data_dir = 'data'
rgb_path = path.join(data_dir, 'rgb.h265')
left_path = path.join(data_dir, 'left.h265')
right_path = path.join(data_dir, 'right.h265')
depth_path = path.join(data_dir, 'depth')

def create_pipeline():
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRGB = pipeline.create(dai.node.ColorCamera)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    encoderRGB = pipeline.create(dai.node.VideoEncoder)
    encoderLeft = pipeline.create(dai.node.VideoEncoder)
    encoderRight = pipeline.create(dai.node.VideoEncoder)

    stereo = pipeline.create(dai.node.StereoDepth)

    outRGB = pipeline.create(dai.node.XLinkOut)
    outLeft = pipeline.create(dai.node.XLinkOut)
    outRight = pipeline.create(dai.node.XLinkOut)
    outDepth = pipeline.create(dai.node.XLinkOut)

    outRGB.setStreamName('outRGB')
    outLeft.setStreamName('outLeft')
    outRight.setStreamName('outRight')
    outDepth.setStreamName('outDepth')

    # Properties
    camRGB.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRGB.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
    monoLeft.setCamera("left")
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoRight.setCamera("right")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

    # stereo settings
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
    stereo.setExtendedDisparity(True)
    stereo.setSubpixel(False) # either extended or subpixel. I don't think you can do both

    # Create encoders, one for each camera, consuming the frames and encoding them using H.264 / H.265 encoding
    encoderRGB.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
    encoderLeft.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
    encoderRight.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)

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

    return pipeline

def write_depths(queue):
    while True:
        depthMsg= queue.get()
        filename = f'{depthMsg.getSequenceNum()}.png'
        print(filename)
        cv2.imwrite(path.join(depth_path, filename), depthMsg.getFrame())

def h265_to_mp4(source, dest):
    subprocess.check_call(
        [
            "ffmpeg",
            "-framerate", 
            "30",
            "-i",
            source,
            "-c",
            "copy",
            dest
        ]
    )

def main():
    pipeline = create_pipeline()

    # Connect to device and start pipeline
    with dai.Device(pipeline) as dev:

        # Output queues will be used to get the encoded data from the outputs defined above
        qRGB = dev.getOutputQueue(name='outRGB', maxSize=30, blocking=True)
        qLeft = dev.getOutputQueue(name='outLeft', maxSize=30, blocking=True)
        qRight = dev.getOutputQueue(name='outRight', maxSize=30, blocking=True)
        qDepth = dev.getOutputQueue(name='outDepth', maxSize=30, blocking=True)
        queue = Queue()
        p = Process(target=write_depths, args=(queue,))
        p.start()

        # The .h264 / .h265 files are raw stream files (not playable yet)
        with open(rgb_path, 'wb') as fileRGB, open(left_path, 'wb') as fileLeft, open(right_path, 'wb') as fileRight:
            print("Press Ctrl+C to stop encoding...")
            while True:
                try:
                    # Empty each queue
                    while qRGB.has():
                        qRGB.get().getData().tofile(fileRGB)

                    while qLeft.has():
                        qLeft.get().getData().tofile(fileLeft)

                    while qRight.has():
                        qRight.get().getData().tofile(fileRight)

                    while qDepth.has():
                        queue.put(qDepth.get())
                except KeyboardInterrupt:
                    # Keyboard interrupt (Ctrl + C) detected
                    p.terminate()
                    break

        h265_to_mp4(rgb_path, path.join(data_dir, 'rgb.mp4'))
        h265_to_mp4(left_path, path.join(data_dir, 'left.mp4'))
        h265_to_mp4(right_path, path.join(data_dir, 'right.mp4'))


if __name__=="__main__":
    main()