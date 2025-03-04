import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs_py import point_cloud2
import numpy as np
from std_msgs.msg import String
import math


class ParallelParking(Node):
    def __init__(self):
        super().__init__('parallel_parking')
        
        # Subscriber to receive point cloud data from LIDAR
        self.pc_sub = self.create_subscription(PointCloud2, '/lidar/point_cloud', self.point_cloud_callback, 10)
        
        # Publisher to send cmd_vel to move the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize a parking status
        self.parking_status = "idle"

    def point_cloud_callback(self, msg):
        """
        Callback function to process the incoming point cloud data and detect the parking space.
        """
        # Convert point cloud data to numpy array
        pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pc_data))

        # Extract the points of interest (points on the right side of the road)
        right_side_points = points[points[:, 0] > 0]  # Assuming the right side is positive x-axis

        # Now find gaps in the point cloud for parking space
        parking_spot_found = self.detect_parking_spot(right_side_points)

        if parking_spot_found:
            self.parking_status = "detected"
            self.get_logger().info("Parking spot detected! Ready to parallel park.")
            self.parallel_park(right_side_points)
        else:
            self.parking_status = "idle"
            self.get_logger().info("No parking spot detected, continuing to drive.")

    def detect_parking_spot(self, points):
        """
        Detect a gap in the point cloud where the vehicle can park.
        Returns True if a parking spot is found.
        """
        # Simple method: look for a gap of sufficient size in the y-axis (width of the road)
        gap_threshold = 2.0  # Threshold for gap width in meters
        free_space = points[:, 1]  # Assuming parking space width is along the y-axis
        gap_start = None
        
        for i in range(1, len(free_space)):
            if abs(free_space[i] - free_space[i-1]) > gap_threshold:
                gap_start = i
                break
        
        # If we find a gap, we assume the vehicle can park
        return gap_start is not None

    def parallel_park(self, points):
        """
        Move the robot to parallel park into the detected spot.
        """
        twist_msg = Twist()

        # Simple parallel parking logic: move in a square trajectory to park
        # Adjusting orientation and position to park based on the detected spot

        # Move the robot to align with the parking spot
        twist_msg.linear.x = 0.2  # Move forward
        self.cmd_vel_pub.publish(twist_msg)

        # Adjust the angle of the robot to park
        twist_msg.angular.z = -0.3  # Adjust turning angle
        self.cmd_vel_pub.publish(twist_msg)

        # Once aligned, back into the spot
        twist_msg.linear.x = -0.2  # Move backwards
        twist_msg.angular.z = 0  # Straighten out
        self.cmd_vel_pub.publish(twist_msg)

        self.get_logger().info("Parking maneuver complete.")


def main(args=None):
    rclpy.init(args=args)
    parking_node = ParallelParking()
    rclpy.spin(parking_node)
    parking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
