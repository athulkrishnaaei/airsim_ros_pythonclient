import open3d as o3d
import pandas as pd

# Load data from CSV
data = pd.read_pcd('/home/athul/test2/airsim_ros2/PythonClient/multirotor/lidar_output.pcd')
points = data[['x', 'y', 'z']].values

# Create point cloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
