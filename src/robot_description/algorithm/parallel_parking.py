#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # Import Bool message type
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class DrivingDirections(Node):
    def __init__(self):
        super().__init__('driving_directions')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera2/points',
            self.point_cloud_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.parking_spot_subscriber = self.create_subscription(
            Bool,
            '/parking_spot_found',
            self.parking_spot_callback,
            10)
        self.parking_spot_detected = False  # Flag to track when the parking spot is found
        self.moving_forward = False  # Initialize moving_forward flag
        self.get_logger().info("Waiting for parking spot signal...")
    
    def parking_spot_callback(self, msg):
        if msg.data:
            self.parking_spot_detected = True
            self.get_logger().info("Parking spot found! Starting parking maneuver...")
            # Wait until point cloud data is available
            while not hasattr(self, 'points'):
                self.get_logger().info("Waiting for valid point cloud data...")
                time.sleep(0.1)
            self.lineup_car()


    def lineup_car(self):
        """Align the vehicle with the other car using point cloud data."""
        self.get_logger().info("Starting car lineup...")
        

        # Keep moving forward and processing point cloud until the vehicle is aligned
        while not self.is_car_aligned():
            self.move_forward()
            time.sleep(0.1)  # Keep moving slowly and checking point cloud
        self.stop_vehicle()
        self.get_logger().info("Vehicle aligned. Proceeding with parking maneuvers.")


    
    def is_car_aligned (self) -> bool:
        """Detects an open parking space from the point cloud data."""
        if len(self.points) == 0:
            return False

        #Parameters for Lineup_car
        # Define bounding box limits
        x_min, x_max = 0.0, 2.6  
        y_min, y_max = -1.0, -0.8  
        z_min, z_max = -0.75, 1.5 

        
        # Filter points that are inside the defined bounding box
        filtered_points = [p for p in self.points if (x_min <= p[0] <= x_max) and 
                                                    (y_min <= p[1] <= y_max) and 
                                                    (z_min <= p[2] <= z_max)]
        
        num_filtered = len(filtered_points)
        self.get_logger().info(f"Filtered points in box: {num_filtered}")

        # Define thresholds
        stop_threshold = 10000  # If fewer than 1 point, stop the vehicle

        # Check if the number of filtered points is below the stop threshold
        if num_filtered > stop_threshold:  
            self.get_logger().info("Lined up the vehicle! Starting parallel parking maneuver.")
            self.stop_vehicle()
            return True

        return False
    
    # def execute_movements(self):
    #     # Wait until parking spot is detected before executing movements
    #     if not self.parking_spot_detected:
    #         self.get_logger().info("Waiting for parking spot...")
    #         return
        
    #     # Define the movements for parallel parking
    #     movements = [
    #         ("Backing up and turning right", -0.3, -0.5, 20),  # ("Action name", linear velocity, angular velocity, duration)
    #         # ("Turning left", 0.2, 0.5, 2),
    #         # ("Moving backward", -0.5, 0.0, 3),
    #         # ("Turning right", 0.2, -0.5, 2),
    #         # ("Backing up and turning left", -0.3, 0.5, 2),
    #         # ("Backing up and turning right", -0.3, -0.5, 2),
    #         # ("Stopping", 0.0, 0.0, 0)
    #     ]
        
    #     # Execute the movements sequentially
    #     for action, linear, angular, duration in movements:
    #         self.get_logger().info(action)
    #         twist = Twist()
    #         twist.linear.x = linear
    #         twist.angular.z = angular
    #         self.publisher.publish(twist)
    #         time.sleep(duration)
        
    #     self.get_logger().info("Parallel parking sequence complete. Stopping the node.")
    #     self.destroy_node()
 
    def move_forward(self, speed: float = 0.4) -> None: 
        """Move the vehicle forward at the specified speed."""
        twist = Twist()
        twist.linear.x = speed  # Move forward at specified speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def point_cloud_callback(self, msg) -> None:
        """Callback to process point cloud data."""
        self.points = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        


    def stop_vehicle(self) -> None:
        """Publishes zero velocity to stop the vehicle."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Vehicle stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = DrivingDirections()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
