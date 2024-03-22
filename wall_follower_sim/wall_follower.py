#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import math
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
        self.laser_msg = None
        self.prev_error = 0

        # a publisher for our line marker
        self.WALL_TOPIC = "/wall"
        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)

    def pd_error(self, error):
        Kp = 10
        Kd = 10
        d_error = self.prev_error - error
        angle = Kp * error + Kd * d_error
        return angle

    def publish_data(self):
        if self.laser_msg == None:
            return
        
        msg = self.laser_msg

        ranges = msg.ranges
        min_angle = msg.angle_min
        angle_increment = msg.angle_increment

        ### ranges[i] corresponds to min_angle + i * angle_increment

        angles = np.array([min_angle + angle_increment*i for i in range(100)])
        distances = ranges

        x = distances * np.cos(angles)
        y = distances * np.sin(angles)

        if self.SIDE == -1:
            x = x[:50]
            y = y[:50]
            angles = angles[:50]
        else:
            x = x[50:]
            y = y[50:]
            angles = angles[50:]

        coefficients = np.polyfit(x, y, 1)
        B1, B0 = coefficients

        y_fitted = np.array([B0 + B1*elem for elem in x])
        VisualizationTools.plot_line(x, y_fitted, self.line_pub, frame="/laser")
        # VisualizationTools.plot_line(x_r, y_r, self.publisher_data)

        ### Find distance
        for i, angle in enumerate(angles):
            if angle > -3.14159 / 2:
                break
        distance_from_wall = distances[i] * math.sin(angle)
        print("distance_from_wall is: ", distance_from_wall)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 1.0
        msg.drive.steering_angle = 0.0

        self.get_logger().info(self.DRIVE_TOPIC)
        self.publisher_data.publish(msg)

    def listen_laser_data(self, msg):
        # Print the range measurements
        self.laser_msg = msg
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

        print("len(msg.intensities) is: ", len(msg.intensities))

        print("len(msg.ranges) is: ", len(msg.ranges))
    

def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()