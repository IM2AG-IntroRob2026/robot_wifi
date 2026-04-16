import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class RobotMonitor(Node):

    def __init__(self):
        super().__init__('robot_monitor')

        # Publisher pour contrôler le robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber pour odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot4/odom',
            self.odom_callback,
            10
        )

        # Variables débit
        self.byte_count = 0
        self.start_time = time.time()

        # Position
        self.x = 0.0
        self.y = 0.0

        # Timer
        self.timer_move = self.create_timer(0.5, self.move_robot)
        self.timer_bandwidth = self.create_timer(1.0, self.compute_bandwidth)

    def odom_callback(self, msg):
        # Récupérer position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Approx taille message
        msg_size = len(str(msg).encode('utf-8'))
        self.byte_count += msg_size

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def compute_bandwidth(self):
        current_time = time.time()
        elapsed = current_time - self.start_time

        if elapsed > 0:
            bandwidth = self.byte_count / elapsed
            kbps = bandwidth / 1024

            self.get_logger().info(
                f"Position (x={self.x:.2f}, y={self.y:.2f}) | Débit: {kbps:.2f} KB/s"
            )

        # reset
        self.byte_count = 0
        self.start_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
