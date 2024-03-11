#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from wall_follower_sim.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        # self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        # self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SCAN_TOPIC = "/scan"
        self.DRIVE_TOPIC = "/drive"
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        self.timer = self.create_timer(1/20, self.publish_data)

        self.subscriber_ls = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listen_laser_data, 10)
        self.publisher_data = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)

    def publish_data(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 1.0
        msg.drive.steering_angle = 0.1

        self.get_logger().info(self.DRIVE_TOPIC)
        self.publisher_data.publish(msg)

    def listen_laser_data(self, msg):
        # Print the range measurements
        print("Range measurements: ", msg.ranges)

        # Print the minimum and maximum angles of the scan
        print("Scan angles: ", msg.angle_min, " to ", msg.angle_max)

        # Print the angular distance between measurements
        print("Angle increment: ", msg.angle_increment)

        # Print the time between measurements
        print("Time increment: ", msg.time_increment)

        # Print the minimum and maximum range of the sensor
        print("Sensor range: ", msg.range_min, " to ", msg.range_max)
        # Print the intensity measurements
        print("Intensity measurements: ", msg.intensities)
    

def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()