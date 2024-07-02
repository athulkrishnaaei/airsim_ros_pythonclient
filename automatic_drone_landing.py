import setup_path
import airsim
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import time
import logging

class SafeLandingDetectionNode(Node):
    def __init__(self):
        super().__init__('safe_landing_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/airsim_node/PX4/lidar/Lidar1',  # Change to your LiDAR topic name
            self.pointcloud_callback,  # Callback function
            10)  # QoS profile depth

        self.subscription  # Prevent unused variable warning
        self.map_points = []  # List to store the entire map points
        self.safe_landing_zones = []

        # Initialize AirSim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.mission_completed = False
        self.start_position = airsim.Vector3r(0, 0, -13)  # Initial position (assuming the initial height is -13)

        # Set up logging
        logging.basicConfig(filename='drone_mission.log', level=logging.INFO, format='%(asctime)s:%(levelname)s:%(message)s')
        self.logger = logging.getLogger()

    def execute_mission(self):
        z = -13  # Negative because AirSim uses NED coordinates
        length = 10  # Length of the rectangular region
        width = 10  # Width of the rectangular region
        speed = 1  # Speed of the drone
        step_size = 2  # Distance between each row/column in the grid

        waypoints = self.generate_waypoints(length, width, step_size, z)

        self.logger.info("Taking off...")
        self.client.takeoffAsync().join()

        self.logger.info("Mapping the rectangular region...")
        for point in waypoints:
            self.client.moveToPositionAsync(point.x_val, point.y_val, point.z_val, speed).join()
            rclpy.spin_once(self, timeout_sec=0.1)  # Process ROS callbacks

        self.logger.info("Survey complete. Keeping drone in altitude hold mode...")
        self.client.hoverAsync().join()  # Keep the drone in altitude hold mode

        self.logger.info("Processing accumulated data to find best landing zone...")
        self.process_accumulated_data()

    def generate_waypoints(self, length, width, step_size, z):
        waypoints = []
        for y in np.arange(-width / 2, width / 2, step_size):
            for x in np.arange(-length / 2, length / 2, step_size):
                waypoints.append(airsim.Vector3r(x, y, z))
            for x in np.arange(length / 2, -length / 2, -step_size):
                waypoints.append(airsim.Vector3r(x, y + step_size, z))
        return waypoints

    def pointcloud_callback(self, msg):
        if self.mission_completed:
            return

        self.logger.info('Received point cloud data')

        # Convert ROS PointCloud2 message to numpy array
        try:
            points = self.pointcloud2_to_numpy(msg)
            self.map_points.append(points)
            self.logger.info(f'Accumulated map points: {len(self.map_points)}')
        except Exception as e:
            self.logger.error(f'Error converting PointCloud2 to numpy array: {e}')
            return

    def process_accumulated_data(self):
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
            o3d.io.write_point_cloud("downsampled_map_point_cloud.pcd", o3d_cloud)
            self.logger.info('Saved the downsampled map point cloud')
        except Exception as e:
            self.logger.error(f'Error downsampling point cloud: {e}')
            return
        
        # Segment the largest planar component from the point cloud
        try:
            plane_model, inliers = o3d_cloud.segment_plane(distance_threshold=0.01,
                                                           ransac_n=3,
                                                           num_iterations=1000)
            self.logger.info(f'Segmented plane with model: {plane_model}')
            self.logger.info(f'Number of inlier points: {len(inliers)}')
        except Exception as e:
            self.logger.error(f'Error segmenting plane: {e}')
            return
        
        try:
            inlier_cloud = o3d_cloud.select_by_index(inliers)
        except Exception as e:
            self.logger.error(f'Error selecting inliers: {e}')
            return
        
        # Check if the detected plane is suitable for landing
        try:
            if self.is_safe_for_landing(inlier_cloud):
                self.logger.info('Safe landing zone detected')
                landing_coordinates = self.get_landing_coordinates(inlier_cloud)
                self.logger.info(f'Safe landing coordinates: {landing_coordinates}')
                self.safe_landing_zones.append(landing_coordinates)
            else:
                self.logger.info('No safe landing zone detected at this position')
        except Exception as e:
            self.logger.error(f'Error checking or logging safe landing zone: {e}')

        # After processing, find the best landing zone and land the drone
        self.find_best_landing_zone()

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

    def is_safe_for_landing(self, cloud):
        # Check if the plane is large and flat enough for safe landing
        min_points = 500  # Minimum number of points in the plane
        max_height_variation = 100  # Maximum allowed height variation in the plane
        
        points = np.asarray(cloud.points)
        if points.shape[0] < min_points:
            self.logger.info(f'Plane rejected: insufficient points ({points.shape[0]} < {min_points})')
            return False
        
        # Calculate the height variation
        height_variation = np.max(points[:, 2]) - np.min(points[:, 2])
        if height_variation > max_height_variation:
            self.logger.info(f'Plane rejected: height variation too large ({height_variation} > {max_height_variation})')
            return False
        
        self.logger.info('Plane accepted as safe landing zone')
        return True

    def get_landing_coordinates(self, cloud):
        # Calculate the centroid of the inlier points for landing
        points = np.asarray(cloud.points)
        centroid = np.mean(points, axis=0)
        return centroid

    def find_best_landing_zone(self):
        if not self.safe_landing_zones:
            self.logger.info('No safe landing zones found during the survey')
            self.initiate_failsafe_landing()
            return

        # For simplicity, let's assume the best landing zone is the first one found
        # You can implement more sophisticated criteria for selecting the best zone
        best_landing_zone = self.safe_landing_zones[0]
        self.logger.info(f'Best landing zone coordinates: {best_landing_zone}')
        self.land_drone(best_landing_zone)

    def land_drone(self, coordinates):
        self.logger.info('Initiating landing sequence')
        self.client.moveToPositionAsync(coordinates[0], coordinates[1], coordinates[2], 1).join()
        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            self.logger.info("Drone is already landed")
        else:
            self.logger.info("Landing drone...")
            self.client.landAsync().join()
            self.logger.info("Drone has landed")
        
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

    def initiate_failsafe_landing(self):
        self.logger.info('Failsafe landing initiated')
        if self.safe_landing_zones:
            self.find_best_landing_zone()
        else:
            self.logger.info('Returning to initial position')
            self.client.moveToPositionAsync(self.start_position.x_val, self.start_position.y_val, self.start_position.z_val, 1).join()
            self.logger.info('Landing at initial position')
            self.client.landAsync().join()
            self.client.armDisarm(False)
            self.client.enableApiControl(False)

def main(args=None):
    rclpy.init(args=args)
    node = SafeLandingDetectionNode()
    node.execute_mission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
