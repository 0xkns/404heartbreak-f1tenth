import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np

class WallFollowingNode(Node):

    def __init__(self):
        # Initialize node
        super().__init__('wall_follower')

        # ROS2 parameters
        scan_topic = 'scan'
        drive_topic = 'drive'

        # Speed and steering limits
        self.v_default = 1.0  # Default speed [m/s]
        self.v_turn = 0.5    # Reduced speed for tight turns [m/s]
        self.max_steering = 0.5  # Max steering angle [rad]

        # Wall-following parameters
        self.lookahead = 15  # Lookahead angle for calculations [degrees]

        # PID parameters
        self.k_p = 0.4
        self.k_d = 0.015
        self.k_i = 0.02
        self.setpoint = 1  # Desired distance to wall [m]

        # Control state
        self.prev_error = 0
        self.accumulated_error = 0

        # ROS2 publishers and subscribers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 1)

    def scan_callback(self, scan_msg):
        # Process laser scan data
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min

        # Calculate distances to wall
        offset = -np.pi / 2 - angle_min
        theta = self.lookahead * np.pi / 180
        a = ranges[int(offset / angle_increment)]
        b = ranges[int((offset + theta) / angle_increment)]
        alpha = np.arctan2(a * np.cos(theta) - b, a * np.sin(theta))
        distance = b * np.cos(alpha)

        # Update PID control
        error = self.setpoint - distance
        steering_angle = (
            self.k_p * error +
            self.k_d * (error - self.prev_error) +
            self.k_i * self.accumulated_error
        )
        self.prev_error = error
        self.accumulated_error += error

        # Adjust velocity based on steering angle (reduce for tight turns)
        speed = self.v_turn if abs(steering_angle) > 0.3 else self.v_default

        # Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))
        self.drive_publisher.publish(drive_msg)

        # Debugging information
        self.get_logger().info(f"Error: {error:.3f}, Steering Angle: {steering_angle:.3f}, Speed: {speed:.2f}")


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollowingNode()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
