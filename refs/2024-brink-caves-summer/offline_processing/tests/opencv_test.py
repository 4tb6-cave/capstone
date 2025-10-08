import os
import numpy as np
import cv2
import matplotlib.pyplot as plt


def displayImage(image_path):
    img = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH) # IMREAD_UNCHANGED also works, anything else truncates to 8 bits
    if img is None:
        print("failed to read image")
        return
    print('Datatype:', img.dtype, '\nDimensions:', img.shape)
    cv2.imshow(image_path, img)
    cv2.waitKey(3000)
    return

def main():
    displayImage('/home/arco3/Desktop/rosbags/cave_depthmap/oak_stereo_image_raw_1719505492.846432983.png')
    return

main()