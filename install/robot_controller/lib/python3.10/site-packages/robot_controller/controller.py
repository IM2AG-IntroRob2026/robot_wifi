import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_robot)

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.2   # avance
        msg.angular.z = 0.0  # tout droit

        self.publisher_.publish(msg)
        self.get_logger().info("Le robot avance")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
