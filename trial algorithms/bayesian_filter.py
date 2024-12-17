# Save this as `bayesian_racing_line.py` in a ROS 2 package
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class BayesianRacingLine(Node):
    def __init__(self):
        super().__init__('bayesian_racing_line')

        # Parameters
        self.declare_parameter('num_particles', 100)
        self.declare_parameter('track_waypoints', [])

        self.num_particles = self.get_parameter('num_particles').value
        self.track_waypoints = np.array(self.get_parameter('track_waypoints').value)

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publications
        self.racing_line_pub = self.create_publisher(Float32MultiArray, '/optimized_racing_line', 10)

        # Initialize particles
        self.particles = np.random.uniform(low=-1.0, high=1.0, size=(self.num_particles, 4))
        self.weights = np.ones(self.num_particles) / self.num_particles

    def odom_callback(self, msg):
        # Extract vehicle state from odometry
        car_state = np.array([msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.twist.twist.linear.x,
                              np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)])

        # Bayesian Filtering
        self.predict()
        self.update(car_state)
        self.resample()

        # Optimize the racing line
        optimized_line = self.optimize_racing_line()

        # Publish the optimized racing line
        racing_line_msg = Float32MultiArray()
        racing_line_msg.data = optimized_line.flatten().tolist()
        self.racing_line_pub.publish(racing_line_msg)

    def predict(self):
        """Predict the next state of particles."""
        noise = np.random.normal(0, 0.1, self.particles.shape)
        self.particles += noise

    def update(self, car_state):
        """Update particle weights based on sensor measurements."""
        distances = np.linalg.norm(self.particles[:, :2] - car_state[:2], axis=1)
        self.weights = np.exp(-distances)  # Gaussian-like weighting
        self.weights /= np.sum(self.weights)

    def resample(self):
        """Resample particles based on their weights."""
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def optimize_racing_line(self):
        """Optimize the racing line based on the particle filter's state."""
        # Use the mean of the particles to estimate the optimal racing line
        mean_state = np.mean(self.particles, axis=0)
        closest_waypoint = self.find_closest_waypoint(mean_state[:2])
        return self.track_waypoints[closest_waypoint:closest_waypoint + 10]

    def find_closest_waypoint(self, position):
        """Find the closest waypoint on the track to the current position."""
        distances = np.linalg.norm(self.track_waypoints - position, axis=1)
        return np.argmin(distances)


def main(args=None):
    rclpy.init(args=args)
    node = BayesianRacingLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
