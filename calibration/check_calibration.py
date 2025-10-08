#!/usr/bin/env python3

import depthai as dai
import os
import cv2
import numpy as np

img_dir = 'images'

def create_pipeline():
    pipeline = dai.Pipeline()

    # Define sources and outputs
    # camRGB = pipeline.create(dai.node.ColorCamera)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    # encoderRGB = pipeline.create(dai.node.VideoEncoder)
    # encoderLeft = pipeline.create(dai.node.VideoEncoder)
    # encoderRight = pipeline.create(dai.node.VideoEncoder)

    # outRGB = pipeline.create(dai.node.XLinkOut)
    outLeft = pipeline.create(dai.node.XLinkOut)
    outRight = pipeline.create(dai.node.XLinkOut)

    # outRGB.setStreamName('outRGB')
    outLeft.setStreamName('outLeft')
    outRight.setStreamName('outRight')

    # Properties
    # camRGB.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    # camRGB.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    monoLeft.setCamera("left")
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P) # or THE_800_P
    monoRight.setCamera("right")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

    # Create encoders, one for each camera, consuming the frames and encoding them using H.264 / H.265 encoding
    # encoderRGB.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
    # encoderLeft.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
    # encoderRight.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)

    # Linking
    # camRGB.video.link(outRGB.input)
    monoLeft.out.link(outLeft.input)
    monoRight.out.link(outRight.input)

    return pipeline

def main():

    cv_file = cv2.FileStorage()
    cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

    stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
    stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
    stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
    stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

    pipeline = create_pipeline()

    # Connect to device and start pipeline
    with dai.Device(pipeline) as dev:

        # Output queues will be used to get the encoded data from the outputs defined above
        # qRGB = dev.getOutputQueue(name='outRGB', maxSize=30, blocking=True)
        qLeft = dev.getOutputQueue(name='outLeft', maxSize=30, blocking=True)
        qRight = dev.getOutputQueue(name='outRight', maxSize=30, blocking=True)

        while True:
            
            if qLeft.has():
                frameL = qLeft.get().getCvFrame()
                frameL2 = cv2.remap(frameL, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                # cv2.imshow("left", frameL)
                cv2.imshow("left rectified", cv2.resize(frameL2, None, None, 0.5, 0.5))

            if qRight.has():
                frameR = qRight.get().getCvFrame()
                frameR2 = cv2.remap(frameR, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                # cv2.imshow("right", frameR)
                cv2.imshow("right rectified", cv2.resize(frameR2, None, None, 0.5, 0.5))

            key = cv2.waitKey(1)

            if key == ord('q'):
                break

if __name__=="__main__":
    main()