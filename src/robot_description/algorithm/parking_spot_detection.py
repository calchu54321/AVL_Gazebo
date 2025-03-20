#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import time
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool  # Import Bool message type

class ParkingSpotDetection(Node):
    def __init__(self):
        super().__init__('parking_spot_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera2/points',
            self.point_cloud_callback,
            1)
        self.subscription
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher for notifying that a parking spot is found
        self.spot_found_publisher = self.create_publisher(Bool, '/parking_spot_found', 10)
        self.moving_forward = True
        self.slowing_down = False  # Initialize slowing_down attribute
        self.super_slowing_down = False  # Initialize super_slowing_down attribute
        self.forward_start_time = time.time()
        self.move_forward()

    def move_forward(self, speed: float = 0.8) -> None: 
        """Move the vehicle forward at the specified speed."""
        twist = Twist()
        twist.linear.x = speed  # Move forward at specified speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def point_cloud_callback(self, msg) -> bool:
        # Stop moving forward after processing the initial time
        if self.moving_forward:
            self.get_logger().info("Starting parking spot detection...")
            self.move_forward()

        # Convert PointCloud2 to a list of points
        points = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        
        if self.detect_open_space(points):
            self.moving_forward = False

    def detect_open_space(self, points) -> bool:
        """Detects an open parking space from the point cloud data."""
        if len(points) == 0:
            return False

        # Define bounding box limits for an open parking space
        x_min, x_max = 0.0, 2.6   # Forward distance to check
        y_min, y_max = -1.0, 1.3    # Length of the car
        z_min, z_max = 0, 1.5    # Height range (ground to ~1.5m)

        # Filter points that are inside the defined bounding box
        filtered_points = [p for p in points if (x_min <= p[0] <= x_max) and 
                                                    (y_min <= p[1] <= y_max) and 
                                                    (z_min <= p[2] <= z_max)]
        
        num_filtered = len(filtered_points)
        self.get_logger().info(f"Filtered points in box: {num_filtered}")

        # Define thresholds
        stop_threshold = 1  # If fewer than 1 point, stop the vehicle
        slow_threshold = 15000 # If fewer than 10000 points, slow down the vehicle
        superslow_threshold = 10000 

        # Check if the number of filtered points is below the stop threshold
        if num_filtered < stop_threshold:  
            self.get_logger().info("Open parking spot detected! Stopping vehicle.")
            self.stop_vehicle()
            return True

        # If fewer than slow_threshold points, slow down
        elif num_filtered < slow_threshold:
            if not self.slowing_down:
                self.get_logger().info("Few points detected. Slowing down.")
                self.slow_down()

        # If fewer than superslow_threshold points, slow down even more
        elif num_filtered < superslow_threshold:
            if not self.super_slowing_down:
                self.get_logger().info("Even fewer points detected. Slowing down more.")
                self.super_slow_down()

        # If more points are detected (potential obstacle), resume normal speed
        elif num_filtered >= slow_threshold:
            if self.slowing_down or self.super_slowing_down:
                self.get_logger().info("Obstacle detected or space cleared. Resuming normal speed.")
                self.resume_driving()

        return False

    def stop_vehicle(self) -> None:
        """Publishes zero velocity to stop the vehicle."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Vehicle stopped.")

        # Publish signal to indicate that a parking spot was found
        self.spot_found_publisher.publish(Bool(data=True))
        self.get_logger().info("Published parking spot found signal.")

        # Allow time for message to send before shutting down
        time.sleep(2)
        self.destroy_node()

    def slow_down(self) -> None:
        """Slows down the vehicle to prepare for parking."""
        self.slowing_down = True
        self.move_forward(speed=0.2)  # Slow down the speed to simulate preparing to park

    def super_slow_down(self) -> None:
        """Slows down the vehicle even more."""
        self.super_slowing_down = True
        self.move_forward(speed=0.1)  # Even slower speed to simulate super slow down

    def resume_driving(self) -> None:
        """Resumes normal speed."""
        self.slowing_down = False
        self.super_slowing_down = False
        self.move_forward(speed=0.8)  # Resume normal speed

def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpotDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
