#!/usr/bin/env python3

# fully vibe coded

# Do the following if under Wayland:
# export XDG_SESSION_TYPE=x11; export QT_QPA_PLATFORM=xcb; export DISPLAY=:0; export WAYLAND_DISPLAY=

import argparse
import os
import numpy as np
import open3d as o3d


def load_transform(path):
    T = np.loadtxt(path, delimiter=',')
    if T.shape != (4, 4):
        raise ValueError(f"{path} is not a 4x4 matrix")
    return T


def load_cloud(clouds_dir, frame_id):
    path = os.path.join(clouds_dir, f"frame{frame_id:05d}.ply")
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    return o3d.io.read_point_cloud(path)


def load_pose(poses_dir, frame_id):
    path = os.path.join(poses_dir, f"pose{frame_id:05d}.csv")
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    return load_transform(path)

def load_odometry(odom_dir, frame_id, prev_T):
    path = os.path.join(odom_dir, f"icp{frame_id:05d}_{(frame_id + 1):05d}.csv")
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    return prev_T @ load_transform(path)

def time_to_color(t):
    """
    Map t in [0,1] → color (blue → green → red)
    """
    return [t, 1 - abs(t - 0.5) * 2, 1 - t]

import argparse
import os
import numpy as np
import open3d as o3d
import csv
import time
from datetime import datetime

class AccumulatorViewer:
    def __init__(self, clouds_dir, poses_dir, odom_dir, start, end, voxel_size=None,
                 color_time=False, no_gtsam=False, csv_file=None):
        self.clouds_dir = clouds_dir
        self.poses_dir = poses_dir
        self.start = start
        self.end = end
        self.voxel_size = voxel_size
        self.color_time = color_time
        self.no_gtsam = no_gtsam
        self.odom_dir = odom_dir
        self.csv_file = csv_file
        self.T = np.identity(4)

        self.current_frame = start
        self.accumulated = None
        self.timestamps = {}

        # Load timestamps from CSV if provided
        if csv_file and os.path.exists(csv_file):
            self.load_timestamps(csv_file)

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("Map Builder", width=1000, height=800)

        self.ctr = self.vis.get_view_control()
        self.ctr.set_constant_z_far(10000)
        self.ctr.set_up([0, 1, 0])

        self.vis.register_key_callback(ord("N"), self.next_frame_manual)  # Manual mode
        self.vis.register_key_callback(ord("V"), self.next_frame_video)   # Video mode
        self.vis.register_key_callback(256, self.quit)

        self.geometry = None
        self.video_mode = False
        self.auto_timing = len(self.timestamps) > 0
        self.prev_time = None

        print(f"Frames: {start} → {end}")
        print("Press N for manual advance, V for video mode, ESC to quit")
        print(f"Timestamp mode: {'auto' if self.auto_timing else 'disabled'}")

        self.add_frame(initial=True)

    def load_timestamps(self, csv_path):
        """Load ID and timestamp pairs from CSV file"""
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                frame_id = int(row['ID'])
                timestamp = float(row['Time'])
                self.timestamps[frame_id] = timestamp

    def calculate_delay(self, current_id):
        """Calculate delay based on timestamp differences"""
        if current_id in self.timestamps:
            if self.prev_time is not None:
                current_ts = self.timestamps[current_id]
                return max(0.01, current_ts - self.prev_time)  # Minimum 10ms
        return None

<<<<<<< HEAD
    def color_by_bounding_box(self, pcd, color_by='x', colormap='viridis'):
=======
    # pick cmap from https://matplotlib.org/stable/users/explain/colors/colormaps.html

    def color_by_bounding_box(self, pcd, color_by='z', colormap='magma'):
>>>>>>> 12aa0fc (add context for video generation script, change cmap algo)
        """Color points based on their position within the bounding box"""
        points = np.asarray(pcd.points)
        aabb = pcd.get_axis_aligned_bounding_box()
        min_bound = aabb.get_min_bound()
        max_bound = aabb.get_max_bound()

        # Get coordinate to color by
        if color_by == 'x':
            coord = points[:, 0]
            min_val, max_val = min_bound[0], max_bound[0]
        elif color_by == 'y':
            coord = points[:, 1]
            min_val, max_val = min_bound[1], max_bound[1]
        else:  # 'z'
            coord = points[:, 2]
            min_val, max_val = min_bound[2], max_bound[2]

        # Normalize to [0, 1]
        normalized = (coord - min_val) / (max_val - min_val)

        # Apply colormap
        import matplotlib.pyplot as plt
        cmap = plt.get_cmap(colormap)
        colors = cmap(normalized)[:, :3]  # Take RGB, ignore alpha

        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    def add_frame(self, initial=False):
        if self.current_frame > self.end:
            print("Reached end frame")
            return

        if initial:
            view_ctl = self.vis.get_view_control()
            # Create a custom axis-aligned bounding box
            min_bound = np.array([-0.1, -0.1, -0.1])  # Set your desired min bounds
            max_bound = np.array([0.2, 0.2, 0.2])    # Set your desired max bounds
            # Create bounding box geometry
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
            self.vis.add_geometry(bbox)
            self.vis.remove_geometry(bbox, reset_bounding_box=False)


        print(f"Adding frame {self.current_frame}")

        pcd = load_cloud(self.clouds_dir, self.current_frame)
        if self.color_time:
            t = (self.current_frame - self.start) / max(1, (self.end - self.start))
            color = time_to_color(t)
            pcd.paint_uniform_color(color)
        else:
            # color = [1, 0, 0]
            # self.vis.update_geometry(geometry)
            # self.vis.update_renderer()         # mark redraw needed
            pcd = self.color_by_bounding_box(pcd)
            pass
        if self.no_gtsam:
            if not initial:
                self.T = load_odometry(self.odom_dir, self.current_frame, self.T)
        else:
            self.T = load_pose(self.poses_dir, self.current_frame)

        pcd.transform(self.T)

        if self.voxel_size is not None:
            pcd = pcd.voxel_down_sample(self.voxel_size)


        if self.accumulated is None:
            self.accumulated = pcd
        else:
            self.accumulated += pcd

        if self.geometry is not None:
            self.vis.remove_geometry(self.geometry, reset_bounding_box=False)

        self.geometry = self.accumulated
        self.vis.add_geometry(self.geometry, reset_bounding_box=False)

        # Update timestamp for delay calculation
        if self.current_frame in self.timestamps:
            self.prev_time = self.timestamps[self.current_frame]

    def next_frame_manual(self, vis):
        if self.current_frame >= self.end:
            print("Done.")
            return False

        self.recolor_all()
        self.current_frame += 1
        self.add_frame()
        return False

    def next_frame_video(self, vis):
        self.video_mode = True
        print("Video mode enabled - automatic playback")

        while self.current_frame < self.end:
            # Calculate delay based on timestamps (requires CSV logic added previously)
            delay = self.calculate_delay(self.current_frame + 1)
            print(delay)
            start = time.time()
            while time.time() < start + delay:
                # time.sleep(delay)
                self.ctr.set_up([0, 1, 0])

            # print(time.time() - ( start + delay))

            if self.current_frame >= self.end:
                break

            self.recolor_all()
            self.current_frame += 1
            self.add_frame()

            # CRITICAL FIX: Allow the window to update during the loop
            vis.update_geometry(self.geometry)
            vis.poll_events()
            vis.update_renderer()

        self.video_mode = False
        print("Video playback complete.")
        while True:
            self.ctr.set_up([0, 1, 0])
        return False

    def recolor_all(self):
        if self.accumulated is None:
            return
        # if not self.color_time:
        #     self.accumulated.paint_uniform_color([0.7, 0.7, 0.7])

    def quit(self, vis):
        vis.close()
        return False

    def run(self):

        # opt = self.vis.get_render_option()
        # opt.point_color_option = o3d.visualization.PointColorOption.YCoordinate

        self.vis.run()
        self.vis.destroy_window()


def main():
    parser = argparse.ArgumentParser(description="Progressive point cloud accumulator with timestamp support")
    parser.add_argument("root", help="Root directory")
    parser.add_argument("-s", "--start", type=int, required=True)
    parser.add_argument("-e", "--end", type=int, required=True)
    parser.add_argument("--csv", type=str, default=None,
                        help="CSV file with ID and timestamps for timing")
    parser.add_argument("--voxel", type=float, default=None,
                        help="Voxel size for downsampling (e.g. 0.01)")
    parser.add_argument("--color-time", action="store_true",
                        help="Color frames by time instead of grey/red")
    parser.add_argument("--no-gtsam", action="store_true",
                        help="Don't use gtsam results, only icp odometry")

    args = parser.parse_args()

    clouds_dir = os.path.join(args.root, "Filtered_Point_Clouds")
    poses_dir = os.path.join(args.root, "poses")
    odom_dir = os.path.join(args.root, "transforms")

    viewer = AccumulatorViewer(
        clouds_dir,
        poses_dir,
        odom_dir,
        args.start,
        args.end,
        voxel_size=args.voxel,
        color_time=args.color_time,
        no_gtsam=args.no_gtsam,
        csv_file=args.csv
    )
    viewer.run()


if __name__ == "__main__":
    main()
