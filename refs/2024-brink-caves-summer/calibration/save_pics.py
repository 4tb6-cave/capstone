#!/usr/bin/env python3

import depthai as dai
import os
import cv2
import numpy as np

img_dir = 'images'

def create_pipeline():
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRGB = pipeline.create(dai.node.ColorCamera)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    # encoderRGB = pipeline.create(dai.node.VideoEncoder)
    # encoderLeft = pipeline.create(dai.node.VideoEncoder)
    # encoderRight = pipeline.create(dai.node.VideoEncoder)

    fps = 30
    camRGB.setFps(fps)
    monoLeft.setFps(fps)
    monoRight.setFps(fps)


    outRGB = pipeline.create(dai.node.XLinkOut)
    outLeft = pipeline.create(dai.node.XLinkOut)
    outRight = pipeline.create(dai.node.XLinkOut)

    outRGB.setStreamName('outRGB')
    outLeft.setStreamName('outLeft')
    outRight.setStreamName('outRight')

    # Properties
    camRGB.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRGB.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P) # 1080_P / 4_K / 12_MP for color camera
    monoLeft.setCamera("left")
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P) # THE_800_P (1280x800 full), THE_720_P (1280x720 cropped), THE_400_P (640x400 binning)
    monoRight.setCamera("right")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)

    # Create encoders, one for each camera, consuming the frames and encoding them using H.264 / H.265 encoding
    # encoderRGB.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.MJPEG)
    # encoderRGB.setLossless(True)
    # encoderLeft.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
    # encoderRight.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)

    # Linking
    camRGB.video.link(outRGB.input)
    monoLeft.out.link(outLeft.input)
    monoRight.out.link(outRight.input)

    return pipeline

def main():
    pipeline = create_pipeline()

    try:
        for d in ["rgb", "left", "right"]:
            os.makedirs(os.path.join(img_dir, d))
    except OSError:
        print("directory already exists")
        if len(os.listdir(os.path.join(img_dir, "left"))) != 0:
            k = input("would you like to delete its contents? (y or n)")
            if k == 'y':
                os.system(f'rm -r {img_dir}/*')
                for d in ["rgb", "left", "right"]:
                    os.makedirs(os.path.join(img_dir, d))
            else:
                return

    # Connect to device and start pipeline
    with dai.Device(pipeline) as dev:

        # Output queues will be used to get the encoded data from the outputs defined above
        qRGB = dev.getOutputQueue(name='outRGB', maxSize=30, blocking=False)
        qLeft = dev.getOutputQueue(name='outLeft', maxSize=30, blocking=False)
        qRight = dev.getOutputQueue(name='outRight', maxSize=30, blocking=False)
       
        num = 0
        # queues = {"rgb": qRGB, "left": qLeft, "right": qRight}
        frames = {}

        print("press s to save images and q to quit")

        while True:
            
            ml = qLeft.get()
            mr = qRight.get()
            mRGB = qRGB.get()
            maxSeq = max(ml.getSequenceNum(), mr.getSequenceNum(), mRGB.getSequenceNum())
            while (ml.getSequenceNum() < maxSeq):
                print("skipping")
                ml = qLeft.get()
            while (mr.getSequenceNum() < maxSeq):
                print("skipping")
                mr = qRight.get()
            while (mRGB.getSequenceNum() < maxSeq):
                print("skipping")
                mRGB = qRGB.get()

            if maxSeq !=  max(ml.getSequenceNum(), mr.getSequenceNum(), mRGB.getSequenceNum()):
                print("missing a frame!")
                continue

            frames["rgb"] = mRGB.getCvFrame()
            frames["right"] = mr.getCvFrame()
            frames["left"] = ml.getCvFrame()

            cv2.imshow("rgb", cv2.resize(frames["rgb"], None, None, 0.5, 0.5))
            cv2.imshow("left", cv2.resize(frames["left"], None, None, 0.5, 0.5))
            cv2.imshow("right", cv2.resize(frames["right"], None, None, 0.5, 0.5))

            key = cv2.waitKey(1)

            if key == ord('q'):
                break

            if key == ord('s'):
                print('saving images')
                for k in frames.keys():
                    cv2.imwrite(os.path.join(img_dir, k, f'{num:02d}.png'), frames[k])
                num += 1

if __name__=="__main__":
    main()