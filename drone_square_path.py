# import airsim
# import pandas as pd
# import numpy as np

# class DroneMission:
#     def __init__(self):
#         self.client = airsim.MultirotorClient()
#         self.client.confirmConnection()
#         self.client.enableApiControl(True)
#         self.client.armDisarm(True)

#     def execute_mission(self):
#         z = -10  # Negative because AirSim uses NED coordinates (North, East, Down)
#         length = 8  # Length of the rectangular region
#         width = 8  # Width of the rectangular region
#         speed = 1  # Speed of the drone
#         step_size = 2  # Distance between each row/column in the grid

#         waypoints = self.generate_waypoints(length, width, step_size, z)

#         # Take off and move to the initial altitude
#         print("Taking off...")
#         self.client.takeoffAsync().join()
#         self.client.moveToZAsync(z, 1).join()

#         all_points = []
#         print("Mapping the rectangular region...")
#         for point in waypoints:
#             self.client.moveToPositionAsync(point.x_val, point.y_val, point.z_val, speed).join()
#             lidar_data = self.client.getLidarData()
#             if lidar_data.point_cloud:
#                 points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
#                 all_points.extend(points.tolist())

#         print("Returning to origin and landing...")
#         self.client.moveToPositionAsync(0, 0, z, speed).join()
#         self.client.landAsync().join()
#         self.client.armDisarm(False)
#         self.client.enableApiControl(False)

#         # Save all collected points to CSV
#         if all_points:
#             df = pd.DataFrame(all_points, columns=['x', 'y', 'z'])
#             df.to_csv('lidar_output.csv', index=False)
#             print(f"Data saved to 'lidar_output.csv', collected {len(all_points)} points.")

#     def generate_waypoints(self, length, width, step_size, z):
#         waypoints = []
#         # Create a grid of waypoints
#         for y in np.arange(-width / 2, width / 2 + step_size, step_size):
#             for x in np.arange(-length / 2, length / 2 + step_size, step_size):
#                 waypoints.append(airsim.Vector3r(x, y, z))
#         return waypoints

# if __name__ == "__main__":
#     mission = DroneMission()
#     mission.execute_mission()

###Moves the drone in a square path and publish lidar data 

import airsim
import numpy as np
import open3d as o3d

class DroneMission:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

    def execute_mission(self):
        z = -8  # Negative because AirSim uses NED coordinates (North, East, Down)
        length = 28  # Length of the rectangular region
        width = 28  # Width of the rectangular region
        speed = 2  # Speed of the drone
        step_size = 2  # Distance between each row/column in the grid

        waypoints = self.generate_waypoints(length, width, step_size, z)

        # Take off and move to the initial altitude
        print("Taking off...")
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(z, 1).join()

        all_points = []
        print("Mapping the rectangular region...")
        for point in waypoints:
            self.client.moveToPositionAsync(point.x_val, point.y_val, point.z_val, speed).join()
            lidar_data = self.client.getLidarData()
            if lidar_data.point_cloud:
                points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
                all_points.extend(points.tolist())

        print("Returning to origin and landing...")
        self.client.moveToPositionAsync(0, 0, z, speed).join()
        self.client.landAsync().join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

        # Convert all collected points to a point cloud and save as PCD
        if all_points:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(all_points))
            o3d.io.write_point_cloud("lidar_output.pcd", pcd)
            print(f"Point cloud saved to 'lidar_output.pcd', collected {len(all_points)} points.")

    def generate_waypoints(self, length, width, step_size, z):
        waypoints = []
        # Create a grid of waypoints
        for y in np.arange(-width / 2, width / 2 + step_size, step_size):
            for x in np.arange(-length / 2, length / 2 + step_size, step_size):
                waypoints.append(airsim.Vector3r(x, y, z))
        return waypoints

if __name__ == "__main__":
    mission = DroneMission()
    mission.execute_mission()
