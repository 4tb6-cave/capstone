## this script will just rectify images
# run after prepare_video I guess

import os
import cv2
import numpy as np
import argparse

left_input_dir = 'left'
right_input_dir = 'right'
left_output_dir = 'left_rect'
right_output_dir = 'right_rect'
stereoMap = '/home/arco3/summer2024/cave_slam_2024/offline_processing/stereoMap.xml'

def rectify_images(source_dir, dest_dir, map_x, map_y):
    images = os.listdir(source_dir)
    images.sort()
    os.mkdir(dest_dir)

    for imgF in images:
        imgRaw = cv2.imread(os.path.join(source_dir, imgF), cv2.IMREAD_UNCHANGED)
        imgRect = cv2.remap(imgRaw, map_x, map_y, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        # cv2.imshow("original", imgRaw)
        # cv2.imshow("calibrated and rectified", imgRect)
        cv2.imwrite(os.path.join(dest_dir, imgF), imgRect)
        print("rectified", imgF)

        # key = cv2.waitKey(1)

    cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser("rectify.py")
    parser.add_argument("--source", nargs='?', help="directory with recorded data (if blank, cwd)")
    args = parser.parse_args()
    if args.source:
        base_dir = args.source
    else:
        base_dir = os.getcwd()

    cv_file = cv2.FileStorage()
    cv_file.open(stereoMap, cv2.FileStorage_READ)

    stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
    stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
    stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
    stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

    rectify_images(os.path.join(base_dir, left_input_dir), os.path.join(base_dir, left_output_dir), stereoMapL_x, stereoMapL_y)
    rectify_images(os.path.join(base_dir, right_input_dir), os.path.join(base_dir, right_output_dir), stereoMapR_x, stereoMapR_y)

main()