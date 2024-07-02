import setup_path
import airsim
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct
import time
import math
import signal

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/airsim_node/PX4/lidar/Lidar1',
            self.topic_callback,
            10
        )
        
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
        self.all_points = []
        self.mission_completed = False

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.execute_mission()

    def topic_callback(self, msg):
        if not self.mission_completed:
            cloud = self.convert_ros_to_o3d(msg)
            if cloud:
                # self.all_points.append(np.asarray(cloud.points))
                self.get_logger().info('Accumulating point cloud data...')

    def convert_ros_to_o3d(self, msg):
        fmt = 'fff'
        point_step = msg.point_step
        field_offsets = {field.name: field.offset for field in msg.fields}
        self.get_logger().info(f'PointCloud2 fields: {field_offsets}')
        
        if 'x' in field_offsets and 'y' in field_offsets and 'z' in field_offsets:
            offset_x = field_offsets['x']
            offset_y = field_offsets['y']
            offset_z = field_offsets['z']
        else:
            self.get_logger().error('PointCloud2 message does not contain x, y, z fields')
            return None

        points = []
        for i in range(msg.width * msg.height):
            x, y, z = struct.unpack_from(fmt, msg.data, offset=i*point_step + offset_x)
            points.append([x, y, z])
        
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.array(points))
        return cloud

    def save_point_clouds_to_pcd(self):
        if len(self.all_points) > 0:
            all_points_array = np.vstack(self.all_points)
            merged_cloud = o3d.geometry.PointCloud()
            merged_cloud.points = o3d.utility.Vector3dVector(all_points_array)
            o3d.io.write_point_cloud('output.pcd', merged_cloud)
            self.get_logger().info('Saved accumulated point cloud data to output.pcd')
        else:
            self.get_logger().info('No point cloud data to save.')
        rclpy.shutdown()

    def execute_mission(self):
        z = -13  # Negative because AirSim uses NED coordinates
        length = 20  # Length of the rectangular region
        width = 20  # Width of the rectangular region
        speed = 1  # Speed of the drone
        step_size = 2  # Distance between each row/column in the grid

        waypoints = []

        # Generate waypoints for the rectangular grid pattern
        for y in np.arange(-width / 2, width / 2, step_size):
            for x in np.arange(-length / 2, length / 2, step_size):
                waypoints.append(airsim.Vector3r(x, y, z))
            for x in np.arange(length / 2, -length / 2, -step_size):
                waypoints.append(airsim.Vector3r(x, y + step_size, z))

        print("Taking off...")
        self.client.takeoffAsync().join()

        print("Mapping the rectangular region...")
        for point in waypoints:
            self.client.moveToPositionAsync(point.x_val, point.y_val, point.z_val, speed).join()
            rclpy.spin_once(self, timeout_sec=0.1)  # Collect data during the flight

        print("Returning to the start point...")
        self.client.moveToPositionAsync(0, 0, z, 5).join()

        print("Landing...")
        self.client.landAsync().join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

        self.mission_completed = True
        self.save_point_clouds_to_pcd()

    def signal_handler(self, sig, frame):
        self.get_logger().info('Signal received, saving point cloud data...')
        self.save_point_clouds_to_pcd()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
