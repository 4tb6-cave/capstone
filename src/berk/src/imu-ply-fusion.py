import os
import glob
import numpy as np
from scipy.spatial.transform import Rotation
import pandas as pd
from plyfile import PlyData, PlyElement

# Read poses.csv into a DataFrame for easy lookup
poses_df = pd.read_csv('poses.csv')

# Get all .ply files from the specified directory
#pc_dir = "/home/admin/capstone/src/record"
pc_dir="/home/berk/Documents/4TB6/capstone/src/record"
pointcloud_files = glob.glob(os.path.join(pc_dir, "*.ply"))

for pc_file in pointcloud_files:
    # Extract timestamp from filename (e.g., "pointcloud_1769057851280.ply")
    base_name = os.path.basename(pc_file)
    ts_str = base_name.split('_')[1].split('.')[0]
    cloud_ts_ms = int(ts_str)
    cloud_ts_s = cloud_ts_ms / 1000.0

    # Find the closest pose in poses.csv
    poses_df['diff'] = abs(poses_df['timestamp'] - cloud_ts_s)
    closest_pose = poses_df.loc[poses_df['diff'].idxmin()]

    # Extract quaternion components (q_w, q_x, q_y, q_z) from pose
    q_w = closest_pose['q_w']
    q_x = closest_pose['q_x']
    q_y = closest_pose['q_y']
    q_z = closest_pose['q_z']

    # Read point cloud data
    data = PlyData.read(pc_file)
    
    # Get vertex DATA (not the element itself) as a structured numpy array
    vertex_data = data['vertex'].data
    
    # Extract XYZ coordinates for rotation
    points_xyz = np.column_stack((vertex_data['x'], vertex_data['y'], vertex_data['z']))
    
    # Create rotation object from quaternion (scipy expects [x, y, z, w])
    rot = Rotation.from_quat([q_x, q_y, q_z, q_w])
    rotated_points = rot.apply(points_xyz)
    
    # Update the actual vertex data in-place
    vertex_data['x'] = rotated_points[:, 0]
    vertex_data['y'] = rotated_points[:, 1]
    vertex_data['z'] = rotated_points[:, 2]
    
    # Save updated point cloud (using original element name 'vertex')
    output_path = os.path.join(os.path.dirname(pc_file), f"rotated_{base_name}")
    PlyData([PlyElement.describe(vertex_data, 'vertex')], text=True).write(output_path)

print(f"Processed {len(pointcloud_files)} point clouds. Rotated files saved in '{pc_dir}'")

