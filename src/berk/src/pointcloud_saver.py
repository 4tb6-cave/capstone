#!/usr/bin/env python3
# ---------------------------------------------------------------
#   pointcloud_saver.py
#
#   Usage:  ros2 run <pkg> pointcloud_saver
#
#   Requires:
#     - rclpy
#     - sensor_msgs
#     - numpy (pip install numpy)
#     - open3d (pip install open3d)   # for PCD writing, optional
# ---------------------------------------------------------------
import os
import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d  # optional – you can write your own binary writer


class CloudSaver(Node):
    def __init__(self, out_dir: str = "clouds"):
        super().__init__("cloud_saver")
        self.out_dir = out_dir
        os.makedirs(self.out_dir, exist_ok=True)

        # Subscribe to the point‑cloud topic (change name if needed)
        self.create_subscription(
            PointCloud2,
            "/cloud_one",
            self.callback,
            10
        )
        self.get_logger().info(f"Saving clouds to {self.out_dir}")

    def callback(self, msg: PointCloud2):
        # Convert ROS point‑cloud to numpy array (x,y,z[,intensity,…])
        points = np.frombuffer(msg.data, dtype=np.float32)
        width = msg.width
        height = msg.height
        if msg.is_bigendian:
            points.byteswap(inplace=True)

        # Reshape:  N × F   (F is number of fields, e.g. 4 for xyz+intensity)
        points = points.reshape((height * width, -1))

        # Build timestamp‑based filename
        ts_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = datetime.datetime.utcfromtimestamp(ts_sec)
        fname = f"cloud_{dt.strftime('%Y%m%d_%H%M%S_%f')}.pcd"
        path = os.path.join(self.out_dir, fname)

        # Write as PCD (Open3D handles the header for you)
        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(points[:, :3])  # only xyz

        if points.shape[1] > 3:
            # If intensity or other fields exist, add them
            cloud_o3d.colors = o3d.utility.Vector3dVector(
                (points[:, 3:] / np.max(points[:, 3:]))  # simple scaling
            )

        o3d.io.write_point_cloud(path, cloud_o3d)
        self.get_logger().info(f"Saved {path}")


def main(args=None):
    rclpy.init(args=args)
    node = CloudSaver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
