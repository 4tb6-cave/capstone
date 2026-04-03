#!/usr/bin/env python3

# fully vibe coded

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


class AccumulatorViewer:
    def __init__(self, clouds_dir, poses_dir, odom_dir, start, end, voxel_size=None, color_time=False, no_gtsam=False):
        self.clouds_dir = clouds_dir
        self.poses_dir = poses_dir
        self.start = start
        self.end = end
        self.voxel_size = voxel_size
        self.color_time = color_time
        self.no_gtsam = no_gtsam
        self.odom_dir = odom_dir
        self.T = np.identity(4)

        self.current_frame = start
        self.accumulated = None

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("Map Builder", width=1000, height=800)

        self.vis.register_key_callback(ord("N"), self.next_frame)
        self.vis.register_key_callback(256, self.quit)

        self.geometry = None

        print(f"Frames: {start} → {end}")
        print("Press N to add next frame, ESC to quit")

        self.add_frame(initial=True)

    def add_frame(self, initial=False):
        if self.current_frame > self.end:
            print("Reached end frame")
            return

        print(f"Adding frame {self.current_frame}")

        try:
            pcd = load_cloud(self.clouds_dir, self.current_frame)
            if self.no_gtsam:
                if not initial:
                    self.T = load_odometry(self.odom_dir, self.current_frame, self.T)
            else:
                self.T = load_pose(self.poses_dir, self.current_frame)
        except Exception as e:
            print(f"Error: {e}")
            return

        pcd.transform(self.T)

        # Optional downsampling
        if self.voxel_size is not None:
            pcd = pcd.voxel_down_sample(self.voxel_size)

        # Determine color
        if self.color_time:
            t = (self.current_frame - self.start) / max(1, (self.end - self.start))
            color = time_to_color(t)
        else:
            color = [1, 0, 0]  # highlight new frame

        pcd.paint_uniform_color(color)

        # Merge
        if self.accumulated is None:
            self.accumulated = pcd
        else:
            self.accumulated += pcd

        # Update visualization
        if self.geometry is not None:
            self.vis.remove_geometry(self.geometry, reset_bounding_box=False)

        self.geometry = self.accumulated
        self.vis.add_geometry(self.geometry, reset_bounding_box=initial)

    def recolor_all(self):
        if self.accumulated is None:
            return

        if self.color_time:
            # do nothing — colors are already meaningful
            return
        else:
            self.accumulated.paint_uniform_color([0.7, 0.7, 0.7])

    def next_frame(self, vis):
        if self.current_frame >= self.end:
            print("Done.")
            return False

        self.recolor_all()

        self.current_frame += 1
        self.add_frame()

        return False

    def quit(self, vis):
        vis.close()
        return False

    def run(self):
        self.vis.run()
        self.vis.destroy_window()


def main():
    parser = argparse.ArgumentParser(description="Progressive point cloud accumulator")
    parser.add_argument("root", help="Root directory")
    parser.add_argument("-s", "--start", type=int, required=True)
    parser.add_argument("-e", "--end", type=int, required=True)

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
        no_gtsam=args.no_gtsam
    )
    viewer.run()


if __name__ == "__main__":
    main()