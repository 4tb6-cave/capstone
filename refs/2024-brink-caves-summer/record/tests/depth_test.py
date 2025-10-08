#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
from os import path

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = True
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)
confout = pipeline.create(dai.node.XLinkOut)

xout.setStreamName("disparity")
confout.setStreamName("confidence")

# Properties
# For mono cameras: THE_800_P (1280x800 full), THE_720_P (1280x720 cropped), THE_400_P (640x400 binning)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
monoRight.setCamera("right")

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
# HIGH_ACCURACY sets confidence threshold to 200 and left right check threshold to 5, while HIGH_DENSITY sets them to 245 and 10. Both turn on LRcheck
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_3x3)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(subpixel)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)
depth.confidenceMap.link(confout.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queue will be used to get the disparity frames from the outputs defined above
    q = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)
    qConf = device.getOutputQueue(name="confidence", maxSize=4, blocking=False)

    while True:
        inDisparity = q.get()  # blocking call, will wait until a new data has arrived
        inConf = qConf.get()
        frame = inDisparity.getFrame()
        # Normalization for better visualization
        cvframe = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)
        cv2.imshow("confidence", inConf.getFrame())

        # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        cvframe = cv2.applyColorMap(cvframe, cv2.COLORMAP_JET)
        cv2.imshow("disparity_color", cvframe)
        timestamp = inDisparity.getTimestamp()
        filename = f'depth_{timestamp.total_seconds()}.png'
        cv2.imwrite(path.join("record/data/depth", filename), cvframe)

        if cv2.waitKey(1) == ord('q'):
            break