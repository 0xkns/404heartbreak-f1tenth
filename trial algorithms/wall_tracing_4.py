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
        self.v_default = 2.0  # Higher default speed for straights [m/s]
        self.v_turn = 0.7     # Reduced speed for sharp turns [m/s]
        self.max_steering = 0.6  # Max steering angle [rad]

        # Wall-following parameters
        self.lookahead = 20  # Larger lookahead angle for smoother control [degrees]
        self.turn_threshold = 0.5  # Threshold for detecting sharp turns

        # PID parameters
        self.k_p = 0.6  # Increased proportional gain
        self.k_d = 0.02
        self.k_i = 0.01
        self.setpoint = 1.0  # Desired distance to wall [m]

        # Control state
        self.prev_error = 0.0
        self.accumulated_error = 0.0

        # ROS2 publishers and subscribers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 1)

    def scan_callback(self, scan_msg):
        # Process laser scan data
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min
        num_ranges = len(ranges)

        # Calculate distances to wall (front-left side)
        offset = -np.pi / 2 - angle_min
        theta = self.lookahead * np.pi / 180

        a_index = int((offset) / angle_increment) % num_ranges
        b_index = int((offset + theta) / angle_increment) % num_ranges

        a = ranges[a_index] if ranges[a_index] > 0.1 else self.setpoint
        b = ranges[b_index] if ranges[b_index] > 0.1 else self.setpoint

        # Angle of deviation
        alpha = np.arctan2(a * np.cos(theta) - b, a * np.sin(theta))
        distance = b * np.cos(alpha)

        # PID control calculations
        error = self.setpoint - distance
        steering_angle = (
            self.k_p * error +
            self.k_d * (error - self.prev_error) +
            self.k_i * self.accumulated_error
        )

        # Update errors
        self.prev_error = error
        self.accumulated_error += error

        # Adjust speed based on sharpness of turn
        speed = self.v_turn if abs(steering_angle) > self.turn_threshold else self.v_default

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
