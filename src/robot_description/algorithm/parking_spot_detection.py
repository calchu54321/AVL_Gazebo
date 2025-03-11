#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class VehicleStopNode(Node):
    def __init__(self):
        super().__init__('vehicle_stop_on_open_space')
        
        # Initialize subscriber to the PointCloud2 topic
        self.point_cloud_subscriber = self.create_subscription(
            PointCloud2,
            '/camera2/points', #topic name
            self.point_cloud_callback,
            10
        )

        # Publisher for controlling the vehicle
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Flag for detecting an open spot
        self.open_space_detected = False

    def point_cloud_callback(self, msg):
        # Convert PointCloud2 message to a list of points
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Convert to a NumPy array for easy manipulation
        points = np.array(list(pc_data))
        
        # Check for an open spot (we'll consider an open spot as points with z > 0)
        # This is just an example; you may need to adjust the criteria based on your setup
        open_space_points = points[points[:, 2] > 0.5]  # Adjust threshold value as needed

        # If there's an open space (i.e., points in front of the car)
        if len(open_space_points) > 0:
            self.open_space_detected = True
            self.get_logger().info("Open space detected, stopping the vehicle.")
            self.stop_vehicle()
        else:
            self.open_space_detected = False

    def stop_vehicle(self):
        # Send a stop command to the vehicle
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = VehicleStopNode()
    rclpy.spin(node)

    # Clean up and shutdown the node when finished
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
