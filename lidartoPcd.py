#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import subprocess

class LidarToPointCloud(Node):
    def __init__(self, bag_file, topic, output_pcd_file):
        super().__init__('lidar_to_pointcloud')
        self.bag_file = bag_file
        self.topic = topic
        self.output_pcd_file = output_pcd_file
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received point cloud data')
        self.convert_and_save(msg)

    def convert_and_save(self, msg):
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        # Convert to Open3D PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        o3d.io.write_point_cloud(self.output_pcd_file, pcd)
        self.get_logger().info(f'PointCloud saved to {self.output_pcd_file}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    bag_file = '/home/athul/test2/airsim_ros2/PythonClient/multirotor/my_bag'
    lidar_topic = '/airsim_node/PX4/lidar/Lidar1'
    output_pcd_file = 'output_file.pcd'


    node = LidarToPointCloud(bag_file, lidar_topic, output_pcd_file)

    # Play the bag file using subprocess
    play_process = subprocess.Popen(['ros2', 'bag', 'play', bag_file])
    
    rclpy.spin(node)

    # Terminate the bag play process after shutting down the node
    play_process.terminate()

if __name__ == '__main__':
    main()
