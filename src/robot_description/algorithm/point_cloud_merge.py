#!/usr/bin/env python3
#Doesn't Work
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class PointCloudMerge(Node):

    def __init__(self):
        super().__init__('point_cloud_merge')

        # Declare a publisher for the merged point cloud
        self.pub = self.create_publisher(PointCloud2, '/merged/point_cloud', 10)

        # Create subscribers for both cameras
        self.create_subscription(PointCloud2, '/camera1/points', self.pc_callback1, 10)
        self.create_subscription(PointCloud2, '/camera2/points', self.pc_callback2, 10)

        # List to store the received point clouds
        self.points = []

    def pc_callback1(self, msg):
        self.get_logger().info('Received point cloud from Camera 1')
        self.process_point_cloud(msg)

    def pc_callback2(self, msg):
        self.get_logger().info('Received point cloud from Camera 2')
        self.process_point_cloud(msg)

    def process_point_cloud(self, msg):
        # Extract point cloud data as a generator of (x, y, z) tuples
        points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Convert points to a list of tuples
        pc_data = list(points)
        
        # Debugging: Check if there are any invalid data points
        if len(pc_data) > 0:
            self.get_logger().info(f"First few points: {pc_data[:5]}")  # Show the first 5 points
        else:
            self.get_logger().warn("No valid points received in the current callback.")

        # Append the current points to the storage buffer
        if len(pc_data) > 0:
            self.points.append(pc_data)

        # Ensure we have at least two point clouds before merging
        if len(self.points) < 2:
            return  # Wait until at least two point clouds are received

        # Merge point clouds (stack them vertically)
        merged_points = np.vstack(self.points)

        # Debugging: Check the structure of merged_points
        self.get_logger().info(f"Merged points shape: {merged_points.shape}")

        # Create a list of tuples to match the PointCloud2 message format
        merged_points_tuples = []
        for point in merged_points:
            # Convert each point from numpy array to tuple (x, y, z)
            merged_points_tuples.append((float(point[0]), float(point[1]), float(point[2])))

        # Create a new PointCloud2 message
        header = msg.header
        header.stamp = self.get_clock().now().to_msg()  # Update timestamp
        header.frame_id = "map"  # Ensure consistent frame ID

        merged_msg = point_cloud2.create_cloud_xyz32(header, merged_points_tuples)

        # Publish the merged point cloud
        self.pub.publish(merged_msg)
        self.get_logger().info(f'Merged point cloud published with {len(merged_points_tuples)} points.')

def main(args=None):
    rclpy.init(args=args)

    point_cloud_merge_node = PointCloudMerge()

    rclpy.spin(point_cloud_merge_node)

    point_cloud_merge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
