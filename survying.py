#Node for moving the drone in a square path and publish the lidar data into a topic 
import setup_path
import airsim
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import logging

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/airsim_node/PX4/lidar/Lidar1',  # Change to your LiDAR topic name
            self.pointcloud_callback,  # Callback function
            10)  # QoS profile depth

        self.subscription  # Prevent unused variable warning

        # Set up logging
        logging.basicConfig(filename='point_cloud_processor.log', level=logging.INFO, format='%(asctime)s:%(levelname)s:%(message)s')
        self.logger = logging.getLogger()

        # Initialize AirSim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

    def pointcloud_callback(self, msg):
        self.logger.info('Received point cloud data')

    def execute_mission(self):
        z = -10  # Negative because AirSim uses NED coordinates
        length = 8  # Length of the rectangular region
        width = 8  # Width of the rectangular region
        speed = 1  # Speed of the drone
        step_size = 2  # Distance between each row/column in the grid

        waypoints = self.generate_waypoints(length, width, step_size, z)

        print("Taking off...")
        self.client.takeoffAsync().join()

        print("Mapping the rectangular region...")
        for point in waypoints:
            self.client.moveToPositionAsync(point.x_val, point.y_val, point.z_val, speed).join()
            rclpy.spin_once(self, timeout_sec=0.1)  # Collect data during the flight

        print("Returning to origin and landing...")
        self.client.moveToPositionAsync(0, 0, z, speed).join()
        self.client.landAsync().join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

    def generate_waypoints(self, length, width, step_size, z):
        waypoints = []
        for y in np.arange(-width / 2, width / 2, step_size):
            for x in np.arange(-length / 2, length / 2, step_size):
                waypoints.append(airsim.Vector3r(x, y, z))
            for x in np.arange(length / 2, -length / 2, -step_size):
                waypoints.append(airsim.Vector3r(x, y + step_size, z))
        return waypoints

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    node.execute_mission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
