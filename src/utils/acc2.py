#!/usr/bin/env python3

import argparse
import os
import numpy as np
import open3d as o3d
o3d.core.sycl.enable_persistent_jit_cache()
import csv
import time
from datetime import datetime

# ... [load_transform, load_cloud, load_pose, load_odometry, time_to_color remain the same] ...

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
        self.geoName = 0

        self.current_frame = start
        self.accumulated = None
        self.timestamps = {}
        
        # Load timestamps from CSV if provided
        if csv_file and os.path.exists(csv_file):
            self.load_timestamps(csv_file)

        print(f"Device API: {o3d.__DEVICE_API__}")
        print(f"SYCL devices: {o3d.core.sycl.get_available_devices()}")

        app = o3d.visualization.gui.Application.instance
        app.initialize()

        self.vis = o3d.visualization.O3DVisualizer("My Window", 800, 600)
        # self.vis.create_window("Map Builder", width=1000, height=800)

        self.vis.scene.scene.add_directional_light(
            "my_light",                    # light name
            [1.0, 1.0, 1.0],              # color (RGB)
            [0.577, -0.577, -0.577],      # direction
            100000.0,                      # intensity
            True                           # cast shadows
        )

        # self.vis.register_key_callback(ord("N"), self.next_frame_manual)  # Manual mode
        # self.vis.register_key_callback(ord("V"), self.next_frame_video)   # Video mode
        # self.vis.register_key_callback(256, self.quit)

        # ctr = self.vis.get_view_control()
        # # Ensure the up direction is fixed (default z-axis)
        # ctr.set_up([0, 1, 0])  # [x, y, z] up vector

        self.geometry = None
        self.video_mode = False
        self.auto_timing = len(self.timestamps) > 0
        self.prev_time = None

        print(f"Frames: {start} → {end}")
        print("Press N for manual advance, V for video mode, ESC to quit")
        print(f"Timestamp mode: {'auto' if self.auto_timing else 'disabled'}")

        self.add_frame(initial=True)

        import threading
        thread = threading.Thread(target=self.next_frame_video, args=([self.vis]))
        thread.daemon = True
        thread.start()

        self.vis.setup_camera(60, [0, 0, 0], [1, 1, 1], [0, 1, 0])

        self.vis.mouse_mode = o3d.visualization.gui.SceneWidget.Controls.FLY
        o3d.visualization.gui.Application.instance.add_window(self.vis)
        self.vis.mouse_mode = o3d.visualization.gui.SceneWidget.Controls.FLY
        o3d.visualization.gui.Application.instance.run()

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
                delay = current_ts - self.prev_time
                print(delay)
                return delay
        return None

    def add_frame(self, initial=False):
        if self.current_frame > self.end:
            print("Reached end frame")
            return

        print(f"Adding frame {self.current_frame}")

        pcd = load_cloud(self.clouds_dir, self.current_frame)
        if self.no_gtsam:
            if not initial:
                self.T = load_odometry(self.odom_dir, self.current_frame, self.T)
        else:
            self.T = load_pose(self.poses_dir, self.current_frame)

        pcd.transform(self.T)

        if self.voxel_size is not None:
            pcd = pcd.voxel_down_sample(self.voxel_size)

        #if self.color_time:
        t = (self.current_frame - self.start) / max(1, (self.end - self.start))
        color = time_to_color(t)
        #else:
        #    color = [1, 0, 0]

        pcd.paint_uniform_color(color)

        if self.accumulated is None:
            self.accumulated = pcd
        else:
            self.accumulated += pcd

        if self.geometry is not None:
            self.vis.remove_geometry("dontcare")

        self.geometry = self.accumulated
        self.vis.add_geometry(str(self.geoName), self.geometry)
        self.geoName += 1
        self.vis.mouse_mode = o3d.visualization.gui.SceneWidget.Controls.FLY

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
        time.sleep(1)
        self.video_mode = True
        print("Video mode enabled - automatic playback")

        while self.current_frame < self.end:
            print(vis.mouse_mode)  # Should print: Controls.FLY
            # Calculate delay based on timestamps (requires CSV logic added previously)
            delay = self.calculate_delay(self.current_frame + 1)
            start = time.time()
            end = time.time()
            while start + delay > end:
                time.sleep(delay)
                end = time.time()
            
            if self.current_frame >= self.end:
                break
            
            self.recolor_all()
            self.current_frame += 1
            self.add_frame()
            
            # CRITICAL FIX: Allow the window to update during the loop
            # vis.update_geometry(self.geometry)
            # vis.poll_events()
            # ctr = self.vis.get_view_control()
            # ctr.set_up([0, 1, 0])  # [x, y, z] up vector
            # vis.update_renderer()
            
        self.video_mode = False
#        time.sleep(10000)
        print("Video playback complete.")
        return False

    def recolor_all(self):
        if self.accumulated is None:
            return
        #if not self.color_time:
        #    self.accumulated.paint_uniform_color([0.7, 0.7, 0.7])

    def quit(self, vis):
        vis.close()
        return False

    def run(self):
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

if __name__ == "__main__":
    main()
