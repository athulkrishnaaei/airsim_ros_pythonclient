import setup_path
import airsim
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
import struct
from std_msgs.msg import Header
import logging

class SafeLandingDetectionNode(Node):
    def __init__(self):
        super().__init__('safe_landing_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            'detected_planes',  # Topic name for detected planes
            self.pointcloud_callback,  # Callback function
            10)  # QoS profile depth

        self.subscription  # Prevent unused variable warning
        self.safe_landing_zones = []

        # Initialize AirSim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.mission_completed = False

        # Set up logging
        logging.basicConfig(filename='drone_mission.log', level=logging.INFO, format='%(asctime)s:%(levelname)s:%(message)s')
        self.logger = logging.getLogger()

    def pointcloud_callback(self, msg):
        if self.mission_completed:
            return

        self.logger.info('Received point cloud data')

        # Convert ROS PointCloud2 message to numpy array
        try:
            points = self.pointcloud2_to_numpy(msg)
            self.logger.info(f'Number of points in detected plane: {points.shape[0]}')

            # Convert numpy array to Open3D point cloud
            o3d_cloud = self.numpy_to_open3d(points)
            self.logger.info('Converted detected plane to Open3D point cloud')

            # Check if the detected plane is suitable for landing
            if self.is_safe_for_landing(o3d_cloud):
                self.logger.info('Safe landing zone detected')
                landing_coordinates = self.get_landing_coordinates(o3d_cloud)
                self.logger.info(f'Safe landing coordinates: {landing_coordinates}')
                self.safe_landing_zones.append(landing_coordinates)

                # Land the drone at the detected safe landing zone
                self.land_drone(landing_coordinates)
                self.mission_completed = True  # Mark mission as completed to stop further processing
            else:
                self.logger.info('No safe landing zone detected at this position')
        except Exception as e:
            self.logger.error(f'Error processing detected plane: {e}')

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

    # def is_safe_for_landing(self, cloud):
    #     # Check if the plane is large and flat enough for safe landing
    #     min_points = 400  # Minimum number of points in the plane
    #     max_height_variation = 270.0  # Maximum allowed height variation in the plane (in meters)
        
    #     points = np.asarray(cloud.points)
    #     if points.shape[0] < min_points:
    #         self.logger.info(f'Plane rejected: insufficient points ({points.shape[0]} < {min_points})')
    #         return False
        
    #     # Calculate the height variation
    #     height_variation = np.max(points[:, 2]) - np.min(points[:, 2])
    #     self.logger.info(f'Plane rejected or not height variation  ({height_variation} > {max_height_variation})')
    #     if height_variation > max_height_variation:
    #         self.logger.info(f'Plane rejected: height variation too large ({height_variation} > {max_height_variation})')
    #         return False
        
    #     self.logger.info('Plane accepted as safe landing zone')
    #     self.logger.info(f'Plane accepted: sufficient points ({points.shape[0]} < {min_points})')
    #     self.logger.info(f'Plane accepted: height variation correct ({height_variation} > {max_height_variation})')
    #     return True
    def is_safe_for_landing(self, cloud):
    # Parameters
        min_points = 330  # Minimum number of points in the plane
        max_height_variation = 13.0  # Maximum allowed height variation in the plane (in meters)
        flying_altitude = 8.0  # Altitude the drone is flying at (in meters)
    
        # Get drone dimensions
        drone_length = 5.0  # Length of the drone (in meters)
        drone_width = 5.0  # Width of the drone (in meters)
        required_landing_area = drone_length * drone_width

        points = np.asarray(cloud.points)
    
        if points.shape[0] < min_points:
            self.logger.info(f'Plane rejected: insufficient points ({points.shape[0]} < {min_points})')
            return False


        # Calculate the height variation
        # height_variation = np.max(points[:, 2]) - np.min(points[:, 2])
        # if height_variation > max_height_variation:
        #     self.logger.info(f'Plane rejected: height variation too large ({height_variation} > {max_height_variation})')
        #     return False

        # Calculate the convex hull and its area
        hull, _ = cloud.compute_convex_hull()
        plane_area = hull.get_surface_area()
        self.logger.info(f'Plane  area  ({plane_area})')
        if plane_area < required_landing_area:
            self.logger.info(f'Plane rejected: insufficient area ({plane_area} < {required_plane_area})')
            return False

        # Ensure the plane is at the correct altitude for landing
        plane_altitude = np.mean(points[:, 2])
        if abs(plane_altitude - flying_altitude) > max_height_variation:
            self.logger.info(f'Plane rejected: altitude variation too large ({plane_altitude} vs {flying_altitude})')
            return False
    
        # Check if the plane is large enough for the drone to land
        plane_length = np.max(points[:, 0]) - np.min(points[:, 0])
        plane_width = np.max(points[:, 1]) - np.min(points[:, 1])
    
        if plane_length < drone_length or plane_width < drone_width:
            self.logger.info(f'Plane rejected: not enough space (length: {plane_length}, width: {plane_width})')
            return False

        # Create a reference point cloud at the drone's altitude
        ref_points = np.array([[x, y, flying_altitude] for x, y in zip(points[:, 0], points[:, 1])])
        ref_cloud = o3d.geometry.PointCloud()
        ref_cloud.points = o3d.utility.Vector3dVector(ref_points)

        # Calculate the distance between the reference point cloud and the detected plane
        dists = np.asarray(ref_cloud.compute_point_cloud_distance(cloud))
        height_variation = np.max(dists)
        self.logger.info(f' height variation  ({height_variation} )')
        if height_variation > max_height_variation:
            self.logger.info(f'Plane rejected: height variation too large ({height_variation} > {max_height_variation})')
            return False
        # Proximity to obstacles (can be implemented if obstacle data is available)
        # This requires additional logic to check surrounding obstacles within a certain radius

        self.logger.info('Plane accepted as safe landing zone')
        self.logger.info(f'Plane accepted: sufficient points ({points.shape[0]} >= {min_points})')
        # self.logger.info(f'Plane accepted: height variation correct ({height_variation} <= {max_height_variation})')
        self.logger.info(f'Plane accepted: correct altitude ({plane_altitude} ~ {flying_altitude})')
        self.logger.info(f'Plane accepted: enough space (length: {plane_length}, width: {plane_width})')
        return True


    def get_landing_coordinates(self, cloud):
        # Calculate the centroid of the inlier points for landing
        points = np.asarray(cloud.points)
        centroid = np.mean(points, axis=0)
        return centroid

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

def main(args=None):
    rclpy.init(args=args)
    node = SafeLandingDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
