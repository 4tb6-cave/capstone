#!/usr/bin/env python
# PointCloud2 color cube
import rospy
import struct
import cv2
import os
# import numpy as np
# import open3d as o3d

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# data_dir = "/home/arco3/Desktop/aug_12_data/data_009/depth"
data_dir = "/home/arco3/Desktop/aug_7_mac/data_010/depth"

def file_key(filename: str):
    i, j, ext = filename.split('.')
    return float(f'{i}.{j}')



def cube():
    points = []
    lim = 8
    for i in range(lim):
        for j in range(lim):
            for k in range(lim):
                x = float(i) / lim
                y = float(j) / lim
                z = float(k) / lim
                pt = [x, y, z, 0]
                r = int(x * 255.0)
                g = int(y * 255.0)
                b = int(z * 255.0)
                a = 255
                print(r, g, b, a)
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                print(hex(rgb))
                pt[3] = rgb
                points.append(pt)
    return points


rospy.init_node("create_cloud_xyzrgb")
pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

depth_scale = 1000 # depth is in mm
fx = 1025 # 2.35 / (0.003) # focal length divided by pixel size
fy = fx # same as fy
cx = 1280 * 0.5
cy = 720 * 0.5
one_over_f = 1 / fx

def from_depth(file):
    depth = cv2.imread(file, cv2.IMREAD_ANYDEPTH)
    # print('Datatype:', depth.dtype, '\nDimensions:', depth.shape)
    # height, width, channels = depth.shape
    # print(height, width, channels)
    depth_filt = cv2.medianBlur(depth, 5)
    # med2 = cv2.medianBlur(depth_filt, 5)

    # cv2.imshow("depth", depth)
    cv2.imshow("depth_filt", depth_filt)
    # cv2.imshow("med2", med2)
    key = cv2.waitKey(100)
    if key == ord('q'):
        raise KeyboardInterrupt
    # print(depth)
    points = []

    h, w = depth.shape

    
    
    for u in range(w):
        for v in range(h):
            d = float(depth_filt[v, u])
            if (d == 0):
                continue
            z = d * 0.001 # 1/1000
            if z > 20 or z < 0.2: # filter a little
                continue
            x = (u - cx) * z * one_over_f
            y = (v - cy) * z * one_over_f
            points.append([x, y, z, 0])

    return points

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 16, PointField.UINT32, 1),
          ]

files = os.listdir(data_dir)
files.sort(key=file_key)

test_f = '38.118005.png'
for f in files:
    points = from_depth(os.path.join(data_dir, f))

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish(pc2)
    # rospy.sleep(0.1)

    # key = cv2.waitKey(50)
            
    # if key == ord('q'):
    #     print("q pressed")
    #     break



# points = cube()
# header = Header()
# header.stamp = rospy.Time.now()
# header.frame_id = "map"
# pc2 = point_cloud2.create_cloud(header, fields, points)
# while not rospy.is_shutdown():
#     pc2.header.stamp = rospy.Time.now()
#     pub.publish(pc2)
#     rospy.sleep(1.0)