#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import time
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class ParkingSpotDetection(Node):
    def __init__(self):
        super().__init__('parking_spot_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera2/points',
            self.point_cloud_callback,
            10)
        self.subscription
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.moving_forward = True
        self.forward_start_time = time.time()
        self.move_forward()

    def move_forward(self) -> None: 
        twist = Twist()
        twist.linear.x = 1.0  # Move forward at 1 m/s
        twist.angular.z = 0.0
        self.publisher.publish(twist)


    def point_cloud_callback(self, msg) -> bool:
        # Ensure at least 1 second of movement before processing
        # if self.moving_forward and (time.time() - self.forward_start_time < 1):
        #     return

        # Stop moving forward after processing the initial time
        if self.moving_forward:
            self.get_logger().info("Starting parking spot detection...")
            self.move_forward()

        # Convert PointCloud2 to a list of points
        points = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        # for p in points: 
        #     self.get_logger().info(f'points X: {p[0]}'.format(p[0]))
        #     self.get_logger().info(f'points Y: {p[1]}'.format(p[1]))
        
        if self.detect_open_space(points):
            self.moving_forward = False

    def detect_open_space(self, points) -> bool:
        """Detects an open parking space from the point cloud data."""
        if len(points) == 0:
            return False
        
        # Example: Check for a clear space within a specific region in front of the vehicle
        x_range = (-1.0, 1.0)  # Left and right boundary in meters
        y_range = (0.5, 2.5)   # Distance ahead to check for an open spot
        
        # filtered_points = [p for p in points if x_range[0] <= p[0] <= x_range[1] and y_range[0] <= p[1] <= y_range[1]]
        

        filtered_points = [p for p in points if x_range[0] <= p[0] <= x_range[1]]
        if len(filtered_points) < 50000:  # If few points are detected, assume it's an open space
            self.get_logger().info("Open parking spot detected! Stopping vehicle.")
            self.stop_vehicle()
            return True
        return False

    def stop_vehicle(self) -> None:
        """Publishes zero velocity to stop the vehicle."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Vehicle stopped.")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpotDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
