import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import logging
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from ament_index_python.packages import get_package_share_directory

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        self.map_points = []  # List to store the entire map points

        # Set up logging
        logging.basicConfig(filename='point_cloud_processor.log', level=logging.INFO, format='%(asctime)s:%(levelname)s:%(message)s')
        self.logger = logging.getLogger()

    def read_rosbag(self, bag_file):
        self.logger.info('Reading ROS bag file')
        
        storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        for topic, data, t in reader.read_messages():
            if topic == '/airsim_node/PX4/lidar/Lidar1':
                msg_type = get_message(type_map[topic])
                msg = msg_type()
                msg.deserialize(data)
                self.pointcloud_callback(msg)
                
    def pointcloud_callback(self, msg):
        self.logger.info('Received point cloud data')

        # Convert ROS PointCloud2 message to numpy array
        try:
            points = self.pointcloud2_to_numpy(msg)
            self.map_points.append(points)
            self.logger.info(f'Accumulated map points: {len(self.map_points)}')
        except Exception as e:
            self.logger.error(f'Error converting PointCloud2 to numpy array: {e}')
            return

    def process_and_save_point_cloud(self):
        # Combine all map points into a single numpy array
        combined_points = np.vstack(self.map_points)
        self.logger.info(f'Combined map point cloud data shape: {combined_points.shape}')

        # Convert numpy array to Open3D point cloud
        try:
            o3d_cloud = self.numpy_to_open3d(combined_points)
            self.logger.info('Converted combined numpy array to Open3D point cloud')
        except Exception as e:
            self.logger.error(f'Error converting numpy array to Open3D point cloud: {e}')
            return
        
        # Downsample the point cloud
        try:
            o3d_cloud = o3d_cloud.voxel_down_sample(0.09)
            self.logger.info('Downsampled the point cloud')

            # Save the downsampled point cloud
            o3d.io.write_point_cloud("point_cloudrosbag.pcd", o3d_cloud)
            self.logger.info('Saved the downsampled map point cloud')
        except Exception as e:
            self.logger.error(f'Error downsampling point cloud: {e}')
            return

    def pointcloud2_to_numpy(self, cloud_msg):
        # Convert ROS PointCloud2 message to numpy array
        fmt = 'fff'  # Format for unpacking
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype)
        points = np.vstack([cloud_arr['x'], cloud_arr['y'], cloud_arr['z']]).T
        return points

    def numpy_to_open3d(self, points):
        # Convert numpy array to Open3D point cloud
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()

    bag_file = '/home/athul/lidar_dataf'  # Update this path to your actual ROS bag file
    node.read_rosbag(bag_file)
    node.process_and_save_point_cloud()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
