#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import math

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)

xlinkOut.setStreamName("imu")

# enable ACCELEROMETER_RAW and GYROSCOPE_RAW
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 200)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
# it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(10)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(20)

# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)

# Pipeline is defined, now we can connect to the device
with dai.Device(pipeline) as device:
    imuType = device.getConnectedIMU()
    imuFirmwareVersion = device.getIMUFirmwareVersion()
    print(f"IMU type: {imuType}, firmware version: {imuFirmwareVersion}")

    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

    with open('imu_data.txt', "w") as file:

        while True:
            imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

            for imuPacket in imuData.packets:
                accel = imuPacket.acceleroMeter
                gyro = imuPacket.gyroscope

                acceleroTs = accel.getTimestampDevice().total_seconds()
                gyroTs = gyro.getTimestampDevice().total_seconds()
                if (acceleroTs != gyroTs):
                    print("acceleration timestamp not equal to gyro timestamp")

                file.write(f"{acceleroTs:.6f} {gyro.x:.8f} {gyro.y:.8f} {gyro.z:.8f} {accel.x:.8f} {accel.y:.8f} {accel.z:.8f}\n")