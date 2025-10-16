#!/usr/bin/env python
# PointCloud2 color cube
# import rospy
import struct
import cv2
import os
import numpy as np
# import open3d as o3d
import argparse

# data_dir = "/home/arco3/Desktop/aug_12_data/data_009/depth"
base_dir = "/home/arco3/Desktop/aug_20/data_011"
# base_dir = "/home/arco3/Desktop/aug_9"
depth_dir = "depth"
left_dir = "left_rect"
right_dir = "right_rect"
depth_2_dir = "depth_2"
rgb_dir = "image"

adjust_params = True
view_debug = True

stereoMap = '/home/arco3/summer2024/cave_slam_2024/offline_processing/stereoMap.xml'
cv_file = cv2.FileStorage()
cv_file.open(stereoMap, cv2.FileStorage_READ)
Q = cv_file.getNode('q_matrix').mat()
# print("Q matrix:\n", Q)


blockSize = 7
num_disp = 80 # should be multiple of 4
stereo = cv2.StereoSGBM_create(
    minDisparity =0,
    numDisparities = num_disp,
    blockSize = blockSize,
    P1 = 8*1*blockSize ** 2,
    P2 = 32*1*blockSize ** 2,
    disp12MaxDiff = 1,
    uniquenessRatio = 10,
    speckleWindowSize = 150,
    speckleRange = 1
    )

blockSize2 = 7
stereo2 = cv2.StereoSGBM_create(
    minDisparity =0,
    numDisparities = num_disp // 2,
    blockSize = blockSize2,
    P1 = 8*1*blockSize2 ** 2,
    P2 = 32*1*blockSize2 ** 2,
    disp12MaxDiff = 1,
    uniquenessRatio = 10,
    speckleWindowSize = 100,
    speckleRange = 1
    )

blockSize3 = 13
stereo3 = cv2.StereoSGBM_create(
    minDisparity =0,
    numDisparities = num_disp // 4,
    blockSize = blockSize3,
    P1 = 8*1*blockSize3 ** 2,
    P2 = 32*1*blockSize3 ** 2,
    disp12MaxDiff = 1,
    uniquenessRatio = 10,
    speckleWindowSize = 50,
    speckleRange = 1
    )

wls_filter = cv2.ximgproc.createDisparityWLSFilter(stereo)
right_matcher = cv2.ximgproc.createRightMatcher(stereo)
    
error_range = 4
erode_size = 13
max_depth = 9999
min_depth = 100

def on_blocksize(val):
    block = val * 2 + 1
    print("block size set to ", block)
    stereo.setBlockSize(block)
    stereo.setP1(8*block**2)
    stereo.setP2(32*block**2)

def on_num_disp(val):
    disp = (val + 1) * 16
    print("max disp set to ", disp)
    stereo.setNumDisparities(disp)

def on_max_diff(val):
    val -= 1
    print("max diff set to ", val)
    stereo.setDisp12MaxDiff(val)

def on_uniqueness(val):
    print("uniqueness ratio set to ", val)
    stereo.setUniquenessRatio(val)

def on_speckle(val):
    print("speckleWindowSize set to ", val)
    stereo.setSpeckleWindowSize(val)

def on_speckle_range(val):
    print("speckle range set to ", val)
    stereo.setSpeckleRange(val)

def on_pfc(val):
    print("pre filter cap set to", val)
    stereo.setPreFilterCap(val)

def on_ddr(val):
    print("ddr set to ", val)
    wls_filter.setDepthDiscontinuityRadius(val)

def on_lambda(val):
    print("lambda set to ", val)
    wls_filter.setLambda(val)

def on_sigma_color(val):
    val = float(val) / 10.
    print("sigma color set to ", val)
    wls_filter.setSigmaColor(val)

def on_range(val):
    global error_range
    error_range = val
    print("range set to", error_range)

def on_es(val):
    global erode_size
    erode_size = val*2 + 1
    print("erode kernel size set to", erode_size)

def on_max_depth(val):
    global max_depth
    max_depth = val
    print("max depth value set to ", val)

def on_min_depth(val):
    global min_depth
    min_depth = val
    print("min depth value set to ", val)

def file_key(filename: str):
    i, j, ext = filename.split('.')
    return float(f'{i}.{j}')

def depth_from_stereo(imgL, imgR):

    imgL2 = cv2.pyrDown(imgL)
    imgR2 = cv2.pyrDown(imgR)
    imgL3 = cv2.pyrDown(imgL2)
    imgR3 = cv2.pyrDown(imgR2)

    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    disp2 = stereo2.compute(imgL2, imgR2).astype(np.float32) / 16.0
    disp3 = stereo3.compute(imgL3, imgR3).astype(np.float32) / 16.0

    # right_disp = right_matcher.compute(imgR, imgL).astype(np.float32) / 16.0
    # disp_filt = wls_filter.filter(disp, left, disparity_map_right=right_disp)
    # conf = wls_filter.getConfidenceMap()

    disp2 = 2 * cv2.resize(disp2, None, None, 2, 2, cv2.INTER_NEAREST_EXACT)
    disp3 = 4 * cv2.resize(disp3, None, None, 4, 4, cv2.INTER_NEAREST_EXACT)

    if view_debug:
        cv2.imshow("disparity raw - blue: 1, green: 1/2, red: 1/4", cv2.merge([disp, disp2, disp3])/num_disp)

    check_1_with_2_3 = cv2.bitwise_and(cv2.inRange(cv2.absdiff(disp, disp2), 0, error_range), cv2.inRange(cv2.absdiff(disp, disp3), 0, error_range))
    check_2_with_3 = cv2.inRange(cv2.absdiff(disp2, disp3), 0, error_range)
    
    # cv2.imshow("check 1", cv2.pyrDown(check_1_with_2_3))
    # cv2.imshow("check 2", cv2.pyrDown(check_2_with_3))

    # disp2Mask = cv2.bitwise_and(check_2_with_3, cv2.bitwise_not(check_1_with_2_3))    
    structuring_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_size, erode_size))
    # d2m_open = cv2.morphologyEx(src=disp2Mask, op=cv2.MORPH_OPEN, kernel=structuring_element)

    d1m_open = cv2.morphologyEx(src=check_1_with_2_3, op=cv2.MORPH_OPEN, kernel=structuring_element)

    if view_debug:
        # blue: excluded from 1, green: excluded from 2, purple: mask 1, yellow: mask 2
        # cv2.imshow("disp masks", cv2.merge([check_1_with_2_3, disp2Mask, cv2.bitwise_or(d1m_open, d2m_open)]))
        cv2.imshow("disp masks", cv2.merge([check_1_with_2_3, check_1_with_2_3, d1m_open]))

    # disp2 = cv2.copyTo(disp2, mask = d2m_open)
    # disp = cv2.copyTo(disp, mask=d1m_open)
    # disp = disp + disp2

    pcl = cv2.reprojectImageTo3D(disparity=disp, Q=Q, ddepth=-1, handleMissingValues=True)
    depth = np.uint16(pcl[:,:,2])

    range_mask = cv2.inRange(depth, min_depth, max_depth)
    if view_debug:
        cv2.imshow("range mask", range_mask)
    # conf_mask = cv2.inRange(conf, confidence_threshold, 255)
    mask = cv2.bitwise_and(range_mask, d1m_open)
    # mask = d1m_open
    mask = cv2.rectangle(mask, pt1=(0,0), pt2=(num_disp, 720), color=0, thickness=cv2.FILLED)

    depth = cv2.copyTo(depth, mask=mask)
    disp = cv2.copyTo(disp, mask=mask)
    # return disp, disp_filt, depth
    return disp, depth

def run(t):
    # rgb_f = f'{t[0]:.6f}.png'
    left_f = f'{t[1]:.6f}.png'
    right_f = f'{t[2]:.6f}.png'
    depth_f = f'{t[3]:.6f}.png'
    left = cv2.imread(os.path.join(base_dir, left_dir, left_f), cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(os.path.join(base_dir, right_dir, right_f), cv2.IMREAD_GRAYSCALE)
    # rgb = cv2.imread(os.path.join(base_dir, rgb_dir, rgb_f), cv2.IMREAD_UNCHANGED)
    # depth_oak = cv2.imread(os.path.join(base_dir, depth_dir, depth_f), cv2.IMREAD_ANYDEPTH)
    # depth_filt = cv2.medianBlur(depth_oak, 5)

    disp, depth = depth_from_stereo(left, right)

    if view_debug:
        cv2.imshow("left", cv2.pyrDown(left))
        # cv2.imshow("right", right)
        # cv2.imshow("depth", cv2.applyColorMap(src=np.uint8(depth/256), colormap=cv2.COLORMAP_JET))
        # cv2.imshow("depth from oak", cv2.pyrDown(depth_oak))
        # cv2.imshow("disp filtered", disp_filt/num_disp)

        cv2.imshow("disparity", cv2.applyColorMap(src=np.uint8(256*disp/num_disp), colormap=cv2.COLORMAP_JET))

        # histSize = 81
        # disp_hist = cv2.calcHist(disp, [0], None, [histSize], (1,histSize), accumulate=False)
        # hist_w = 512
        # hist_h = 400
        # bin_w = int(round( hist_w/histSize ))
        # histImage = np.zeros((hist_h, hist_w), dtype=np.uint8)
        # cv2.normalize(disp_hist, disp_hist, alpha=0, beta=hist_h, norm_type=cv2.NORM_MINMAX)
        # for i in range(1, histSize):
        #     cv2.line(histImage, ( bin_w*(i-1), hist_h - int(disp_hist[i-1]) ),
        #             ( bin_w*(i), hist_h - int(disp_hist[i]) ),
        #             255, thickness=2)

        # cv2.imshow("disparity histogram", histImage)


    # size = np.shape(depth)
    # print(size)
    # print(np.shape(rgb))
    # zeros = np.zeros(rgb.shape[:2], dtype="uint8")
    # merged = cv2.merge([np.uint8(disp*255/num_disp), cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), np.uint8(disp*255/num_disp)])
    # cv2.imshow("rgb + depth", cv2.addWeighted(rgb, 0.2, cv2.cvtColor(np.uint8(depth/255), cv2.COLOR_GRAY2RGB), 0.8, 0.0))
    # cv2.imshow("rgb + depth", merged)

    cv2.imwrite(os.path.join(base_dir, "depth_2", depth_f), depth)
    if not view_debug:
        print("saved depth file", depth_f)
    # cv2.imwrite(os.path.join(base_dir, "left_2", depth_f), cv2.pyrDown(left))    

def main():
    parser = argparse.ArgumentParser("opencv_depth.py")
    parser.add_argument("data_path", help="directory where data is stored")
    parser.add_argument("--view", default=False, action=argparse.BooleanOptionalAction, help="show windows")
    parser.add_argument("--tune", default=False, action=argparse.BooleanOptionalAction, help="show sliders")
    args = parser.parse_args()

    global adjust_params, view_debug, base_dir
    adjust_params = args.tune
    view_debug = args.view
    base_dir = args.data_path

    print("view:", args.view)
    print("tune:", args.tune)
    print("data path:", args.data_path)

    if adjust_params and view_debug:
        cv2.namedWindow("disparity")
        cv2.createTrackbar("blocksize", "disparity", 3, 10, on_blocksize)
        cv2.createTrackbar("num_disp", "disparity", 3, 20, on_num_disp)
        cv2.createTrackbar("max diff", "disparity", 2, 5, on_max_diff)
        cv2.createTrackbar("uniqueness ratio", "disparity", 10, 20, on_uniqueness)
        cv2.createTrackbar("speckle window", "disparity", 150, 200, on_speckle)
        cv2.createTrackbar("speckle range", "disparity", 1, 4, on_speckle_range)
        cv2.createTrackbar("preFilterCap", "disparity", 0, 4, on_pfc)

        # cv2.namedWindow("disp filtered")
        # cv2.createTrackbar("ddr", "disp filtered", 0, 10, on_ddr)
        # cv2.createTrackbar("lambda", "disp filtered", 8000, 10000, on_lambda)
        # cv2.createTrackbar("sigma color", "disp filtered", 15, 30, on_sigma_color)

        cv2.namedWindow("disp masks")
        cv2.createTrackbar("erode kernel size", "disp masks", 6, 20, on_es)
        cv2.createTrackbar("error range", "disp masks", 4, 80, on_range)

        cv2.namedWindow("range mask")
        cv2.createTrackbar("max depth", "range mask", 9999, 20000, on_max_depth)
        cv2.createTrackbar("min depth", "range mask", 100, 1000, on_min_depth)


    times = np.loadtxt(os.path.join(base_dir, "time.txt"), skiprows=2)
    os.makedirs(os.path.join(base_dir, depth_2_dir), exist_ok=True)

    for t in times:

        if view_debug:
            key = cv2.waitKey(1)

            if key == ord('q'):
                raise KeyboardInterrupt

            if key == ord('s'):
                continue

        run(t)

if __name__ == "__main__":
    main()