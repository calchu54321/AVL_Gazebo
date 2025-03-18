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
            20)
        self.subscription
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.moving_forward = True
        self.forward_start_time = time.time()
        self.move_forward()

    def move_forward(self) -> None: 
        twist = Twist()
        twist.linear.x = 0.8  # Move forward at 1 m/s
        twist.angular.z = 0.0
        self.publisher.publish(twist)


    def point_cloud_callback(self, msg) -> bool:
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

        # Define bounding box limits for an open parking space
        x_min, x_max = 0.0, 2.6   # Forward distance to check
        y_min, y_max = -1.0, 0.95    # length of the car
        z_min, z_max = 0, 1.5    # Height range (ground to ~1.5m)

        # Filter points that are inside the defined bounding box
        filtered_points = [p for p in points if (x_min <= p[0] <= x_max) and 
                                                    (y_min <= p[1] <= y_max) and 
                                                    (z_min <= p[2] <= z_max)]
        
        num_filtered = len(filtered_points)
        self.get_logger().info(f"Filtered points in box: {num_filtered}")

        # Define a threshold: If too many points are in the box, it's occupied
        threshold = 1  # Adjust based on testing
        if num_filtered < threshold:  
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
