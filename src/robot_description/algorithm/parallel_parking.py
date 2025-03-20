#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # Import Bool message type
import time

class DrivingDirections(Node):
    def __init__(self):
        super().__init__('driving_directions')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.parking_spot_subscriber = self.create_subscription(
            Bool,
            '/parking_spot_found',
            self.parking_spot_callback,
            10)
        self.parking_spot_detected = False  # Flag to track when the parking spot is found
        self.get_logger().info("Waiting for parking spot signal...")
    
    def parking_spot_callback(self, msg):
        if msg.data:
            self.parking_spot_detected = True
            self.get_logger().info("Parking spot found! Starting parking maneuver...")
            self.execute_movements()

    def execute_movements(self):
        # Wait until parking spot is detected before executing movements
        if not self.parking_spot_detected:
            self.get_logger().info("Waiting for parking spot...")
            return
        
        # Define the movements for parallel parking
        movements = [
            ("Moving forward", 0.5, 0.0, 20),  # ("Action name", linear velocity, angular velocity, duration)
            ("Turning left", 0.2, 0.5, 2),
            ("Moving backward", -0.5, 0.0, 3),
            ("Turning right", 0.2, -0.5, 2),
            ("Backing up and turning left", -0.3, 0.5, 2),
            ("Backing up and turning right", -0.3, -0.5, 2),
            ("Stopping", 0.0, 0.0, 0)
        ]
        
        # Execute the movements sequentially
        for action, linear, angular, duration in movements:
            self.get_logger().info(action)
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)
            time.sleep(duration)
        
        self.get_logger().info("Parallel parking sequence complete. Stopping the node.")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DrivingDirections()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
