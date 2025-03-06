#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs_py import point_cloud2
import numpy as np
import time

class ParallelParking(Node):
    def __init__(self):
        super().__init__('parallel_parking')

        # Subscriber to receive 3D point cloud data from the depth camera
        self.pc_sub = self.create_subscription(PointCloud2, '/depth_camera/point_cloud', self.point_cloud_callback, 10)

        # Publisher to send movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parking status
        self.parking_status = "idle"

    def point_cloud_callback(self, msg):
        """
        Processes incoming 3D point cloud data and detects parking spaces.
        """
        # Convert point cloud to numpy array
        pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pc_data))

        # Filter only points on the right side of the vehicle (assuming +X is forward)
        right_side_points = points[points[:, 0] > 0]

        # Detect parking spot
        parking_spot = self.detect_parking_spot(right_side_points)

        if parking_spot is not None:
            self.parking_status = "detected"
            self.get_logger().info("✅ Parking spot detected! Starting parking maneuver.")
            self.parallel_park(parking_spot)
        else:
            self.parking_status = "idle"
            self.get_logger().info("🚗 No parking spot detected, continuing to drive.")

    def detect_parking_spot(self, points):
        """
        Detects a parking spot by checking for a sufficiently large gap in the x-direction
        while ensuring enough width (y-axis) and depth (z-axis) clearance.
        """
        min_parking_length = 2.5  # Minimum space required along x-axis (length of car)
        min_parking_width = 1.5   # Minimum width in y-axis (enough space to fit car)
        min_depth_clearance = 1.0 # Minimum clearance in z-axis (avoid hitting obstacles)

        if len(points) == 0:
            return None

        # Sort points by x-axis (along the road)
        points = points[points[:, 0].argsort()]

        # Extract X, Y, Z positions
        x_positions = points[:, 0]  # Forward distance (along the road)
        y_positions = points[:, 1]  # Side distance (width of space)
        z_positions = points[:, 2]  # Depth distance (check for obstacles)

        # Find gaps in x (long enough for parking)
        for i in range(len(x_positions) - 1):
            gap = x_positions[i + 1] - x_positions[i]
            
            if gap >= min_parking_length:
                # Check if the width (y) and depth (z) are also sufficient
                y_clearance = np.max(y_positions[i:i+2]) - np.min(y_positions[i:i+2])
                z_clearance = np.max(z_positions[i:i+2]) - np.min(z_positions[i:i+2])

                if y_clearance >= min_parking_width and z_clearance >= min_depth_clearance:
                    return x_positions[i]  # Return the starting x position of the parking spot

        return None  # No valid spot found


    def parallel_park(self, parking_spot):
        """
        Performs a parallel parking maneuver into the detected parking spot.
        """
        min_y, max_y, min_z, max_z = parking_spot
        self.get_logger().info(f"🚦 Parking spot dimensions: Width={max_y - min_y:.2f}m, Depth={max_z - min_z:.2f}m")

        # Initialize current velocities
        target_linear_velocity = 0.2  # Target forward velocity
        target_angular_velocity = -0.5  # Target angular velocity (right turn)
        
        smoothing_factor_linear = 0.05  # Adjust this for linear smoothing speed
        smoothing_factor_angular = 0.1  # Adjust this for angular smoothing speed

        current_linear_velocity = 0.0
        current_angular_velocity = 0.0

        twist_msg = Twist()

        # Step 1: Move forward slightly to align
        while abs(current_linear_velocity - target_linear_velocity) > 0.05:
            if current_linear_velocity < target_linear_velocity:
                current_linear_velocity += smoothing_factor_linear
            elif current_linear_velocity > target_linear_velocity:
                current_linear_velocity -= smoothing_factor_linear

            twist_msg.linear.x = current_linear_velocity
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)  # Control loop interval

        # Step 2: Start turning into the spot (smooth turning)
        while abs(current_angular_velocity - target_angular_velocity) > 0.05:
            if current_angular_velocity < target_angular_velocity:
                current_angular_velocity += smoothing_factor_angular
            elif current_angular_velocity > target_angular_velocity:
                current_angular_velocity -= smoothing_factor_angular

            twist_msg.linear.x = target_linear_velocity  # Keep moving forward
            twist_msg.angular.z = current_angular_velocity
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        # Step 3: Reverse into the spot (smooth reversing)
        target_linear_velocity = -0.2  # Set reverse velocity
        target_angular_velocity = 0.5  # Turn left

        while abs(current_linear_velocity - target_linear_velocity) > 0.05:
            if current_linear_velocity < target_linear_velocity:
                current_linear_velocity += smoothing_factor_linear
            elif current_linear_velocity > target_linear_velocity:
                current_linear_velocity -= smoothing_factor_linear

            twist_msg.linear.x = current_linear_velocity
            twist_msg.angular.z = target_angular_velocity
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        # Step 4: Straighten out and stop
        target_linear_velocity = -0.1  # Slow down slightly to straighten out
        target_angular_velocity = 0.0  # Stop turning

        while abs(current_linear_velocity - target_linear_velocity) > 0.05 or abs(current_angular_velocity - target_angular_velocity) > 0.05:
            if current_linear_velocity < target_linear_velocity:
                current_linear_velocity += smoothing_factor_linear
            elif current_linear_velocity > target_linear_velocity:
                current_linear_velocity -= smoothing_factor_linear

            if current_angular_velocity < target_angular_velocity:
                current_angular_velocity += smoothing_factor_angular
            elif current_angular_velocity > target_angular_velocity:
                current_angular_velocity -= smoothing_factor_angular

            twist_msg.linear.x = current_linear_velocity
            twist_msg.angular.z = current_angular_velocity
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        # Stop the vehicle after parking
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        self.get_logger().info("🎯 Parking maneuver complete.")


def main(args=None):
    rclpy.init(args=args)
    parking_node = ParallelParking()
    rclpy.spin(parking_node)
    parking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
