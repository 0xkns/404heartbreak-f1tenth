#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


desired_trajectory = 1
vel = 30

pub = rospy.Publisher('error', pid_input, queue_size=10)


##	Input: 	data: Lidar scan data
##			theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta
def getRange(data, theta):
    # Find the index of the arary that corresponds to angle theta.
    # Return the lidar scan value at that index
    # Do some error checking for NaN and ubsurd values
    ## Your code goes here

    return


def callback(data):
    theta = 50;
    a = getRange(data, theta)
    b = getRange(data, 0)
    swing = math.radians(theta)

    # !/usr/bin/env python3


    class WallFollowingNode(Node):


        def __init__(self):
            ### ROS2 PARAMETERS ###
            scan_topic = "scan"
            # drive_topic = "/nav/drive"
            drive_topic = "drive"

            ### SPEED AND STEERING LIMITS ###
            self.v_forward = 1.0  # [m/s]
            self.max_steering = 0.5  # [rad]

            ### Wallfollower parameters ###
            self.lookahead = 15

            ### PID Parameters ###
            self.k_p = 0.4
            self.k_d = 0.02
            self.k_i = 0.015
            self.setpoint = 1

            ### Control State ###
            self.prev_error = 0
            self.accumulated_error = 0

            ### ROS2 NODE ###
            super().__init__("wall_follower")
            # Drive Publisher
            self.drive_msg = AckermannDriveStamped()
            self.drive_publisher = self.create_publisher(
                AckermannDriveStamped, drive_topic, 1
            )
            # Scan Subscriber
            self.scan_subscriber = self.create_subscription(
                LaserScan, scan_topic, self.scan_callback, 1
            )
            self.scan_subscriber  # prevent unused variable warning

        def scan_callback(self, scan_msg):
            ranges = np.array(scan_msg.ranges)
            angle_increment = scan_msg.angle_increment
            angle_min = scan_msg.angle_min
            angle_max = scan_msg.angle_max

            # Find distance to wall
            offset = - np.pi / 2 - angle_min
            theta = self.lookahead * np.pi / 180
            a = ranges[int(offset / angle_increment)]
            b = ranges[int((offset + theta) / angle_increment)]
            alpha = np.arctan2(a * np.cos(theta) - b, a * np.sin(theta))
            distance = b * np.cos(alpha)

            # Update PID
            error = self.setpoint - distance
            steering_angle = self.k_p * error + self.k_d * self.prev_error + self.k_i * self.accumulated_error
            self.prev_error = error
            self.accumulated_error += error

            # Publish Drive Message
            self.drive_msg.drive.speed = self.v_forward
            self.drive_msg.drive.steering_angle = steering_angle
            self.drive_publisher.publish(self.drive_msg)

    def main(args=None):
        rclpy.init(args=args)

        wallFollower = WallFollowingNode()

        rclpy.spin(wallFollower)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        wallFollower.destroy_node()
        rclpy.shutdown()

    if __name__ == "__main__":
        main()

    msg = pid_input()
    msg.pid_error = error
    msg.pid_vel = vel
    pub.publish(msg)


if __name__ == '__main__':
    print("Laser node started")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()