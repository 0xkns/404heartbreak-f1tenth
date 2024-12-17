import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np


class WallFollow(Node):

    def __init__(self):
        super().__init__("wall_follow")
        self.sub_scan = self.create_subscription(
            LaserScan, 
            "/scan", 
            self.scan_callback, 
            1
        )

        self.sub_odom = self.create_subscription(
            Odometry, 
            "/ego_racecar/odom", 
            self.odom_callback, 
            1
        )

        self.pub_drive = self.create_publisher(
            AckermannDriveStamped, 
            "/drive", 
            10
        )

        self.drive_msg = AckermannDriveStamped()

        self.Kp = 0.400
        self.Ki = 0.020
        self.Kd = 0.015

        self.integral = 0.0
        self.prev_error_1 = 0.0
        self.prev_secs = 0.0
        self.prev_nsecs = 0.0
        self.start_time = time.time()

        self.longitudinal_vel = 0
        self.wheel_base = 0.33

    def getRange(self, scan_data, angle):
        ranges = scan_data.ranges
        angle_rad = angle * (np.pi / 180)
        index = int(abs(angle_rad - scan_data.angle_max) / scan_data.angle_increment)

        return ranges[index]

    def odom_callback(self, odom_data):
        self.longitudinal_vel = odom_data.twist.twist.linear.x

    def scan_callback(self, scan_data):
        secs = scan_data.header.stamp.sec
        nsecs = scan_data.header.stamp.nanosec

        angle_b = 90
        angle_a = 40

        theta = (angle_b - angle_a) * (np.pi / 180)
        distance_b = self.getRange(scan_data, angle_b)  
        distance_a = (
            self.getRange(scan_data, angle_a)
        ) 

        alpha = -1 * np.arctan2(
            (distance_a * np.cos(theta) - distance_b), (distance_a * np.sin(theta))
        )

        actual_distance = distance_b * np.cos(alpha)
        desired_distance = 1.15

        error = desired_distance - actual_distance
        lookahead_distance = 0.1 + self.longitudinal_vel * 0.15

        error_1 = error + lookahead_distance * np.sin(alpha)

        if (
            (self.prev_secs == 0.0)
            & (self.prev_nsecs == 0.0)
            & (self.prev_error_1 == 0.0)
        ):
            self.prev_secs = secs
            self.prev_nsecs = nsecs
            self.prev_error_1 = error_1

        dt = secs - self.prev_secs + (nsecs - self.prev_nsecs) * 1e-9
        derivative = (error_1 - self.prev_error_1) / dt if dt > 0 else 0.0

        try:
            self.integral += error_1 * dt

            steering_angle = (
                (self.Kp * error_1)
                + (self.Ki * self.integral)
                + (self.Kd * derivative)
            )

            if steering_angle < -0.4:
                steering_angle = -0.4
            elif steering_angle > 0.4:
                steering_angle = 0.4
            self.drive_msg.drive.steering_angle = steering_angle

            steering_angle_degrees = abs(steering_angle * (180 / np.pi))

            self.prev_error_1 = error_1
            self.prev_secs = secs
            self.prev_nsecs = nsecs

            speed = 8.5 if abs(steering_angle) < 0.18 else 6.5
            self.drive_msg.drive.speed = speed

            if time.time() - self.start_time < 60:
                self.get_logger().info(
                    f"steering_angle: {steering_angle:.2f} | speed: {self.longitudinal_vel:.2f}"
                )
            self.pub_drive.publish(self.drive_msg)
            
        except ZeroDivisionError:
            pass


def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    try:    
        rclpy.spin(wall_follow)
    except KeyboardInterrupt:
        wall_follow.get_logger().info('Shutting down gracefully...')
    finally:
        # wall_follow.stop_car()
        wall_follow.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()
