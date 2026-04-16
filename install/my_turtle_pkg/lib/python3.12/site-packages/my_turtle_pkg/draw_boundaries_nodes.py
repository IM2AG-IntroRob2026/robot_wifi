#!/usr/bin/env python3
import math
import sys
import threading
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class DrawBoundariesNode(Node):
    def __init__(self):
        super().__init__('draw_boundaries_node')

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_pen service...")

        self.pose = None
        self.start_pose = None
        self.border_start_pose = None

        self.state = "AUTO_FORWARD"
        self.prev_auto_state = "AUTO_FORWARD"

        self.wall_margin = 0.7
        self.linear_speed = 1.5
        self.angular_speed = 1.5

        self.manual_cmd = Twist()

        # Au début : stylo levé
        self.set_pen(off=1)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.stop_keyboard = False
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info("SPACE: manuel / reprise auto | zqsd pour bouger")

    def pose_callback(self, msg):
        self.pose = msg
        if self.start_pose is None:
            self.start_pose = msg

    def control_loop(self):
        if self.pose is None:
            return

        if self.state == "MANUAL":
            self.cmd_pub.publish(self.manual_cmd)
            return

        if self.state == "AUTO_FORWARD":
            self.auto_forward()

        elif self.state == "AUTO_BORDER":
            self.auto_border()

        elif self.state == "AUTO_RETURN":
            self.auto_return()

    def auto_forward(self):
        if self.is_near_wall():
            self.stop_turtle()
            self.border_start_pose = self.copy_pose(self.pose)
            self.set_pen(off=0)   # stylo baissé
            self.state = "AUTO_BORDER"
            self.get_logger().info("Mur atteint -> suivi du contour")
            return

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        self.cmd_pub.publish(cmd)

    def auto_border(self):
        side = self.closest_wall()

        cmd = Twist()

        # Suivi simple du rectangle dans le sens horaire
        if side == "left":
            target_theta = math.pi / 2      # vers le haut
        elif side == "top":
            target_theta = 0.0              # vers la droite
        elif side == "right":
            target_theta = -math.pi / 2     # vers le bas
        else:  # bottom
            target_theta = math.pi          # vers la gauche

        err = self.normalize_angle(target_theta - self.pose.theta)

        cmd.angular.z = 3.0 * err
        if abs(err) < 0.3:
            cmd.linear.x = self.linear_speed
        else:
            cmd.linear.x = 0.2

        self.cmd_pub.publish(cmd)

        # Si on revient près du point de départ du tracé -> contour fermé
        if self.border_start_pose is not None:
            d = self.distance(self.pose, self.border_start_pose)
            if d < 0.3:
                self.stop_turtle()
                self.set_pen(off=1)   # stylo levé
                self.state = "AUTO_RETURN"
                self.get_logger().info("Contour fermé -> retour au point initial")

    def auto_return(self):
        if self.start_pose is None:
            return

        dx = self.start_pose.x - self.pose.x
        dy = self.start_pose.y - self.pose.y
        dist = math.hypot(dx, dy)

        if dist < 0.2:
            self.stop_turtle()
            self.get_logger().info("Retour terminé")
            return

        target_theta = math.atan2(dy, dx)
        err = self.normalize_angle(target_theta - self.pose.theta)

        cmd = Twist()
        if abs(err) > 0.2:
            cmd.angular.z = 2.0 * err
        else:
            cmd.linear.x = min(1.5, dist)
            cmd.angular.z = 2.0 * err

        self.cmd_pub.publish(cmd)

    def is_near_wall(self):
        return (
            self.pose.x < 0.5 + self.wall_margin or
            self.pose.x > 10.5 - self.wall_margin or
            self.pose.y < 0.5 + self.wall_margin or
            self.pose.y > 10.5 - self.wall_margin
        )

    def closest_wall(self):
        distances = {
            "left": abs(self.pose.x - 0.5),
            "right": abs(self.pose.x - 10.5),
            "bottom": abs(self.pose.y - 0.5),
            "top": abs(self.pose.y - 10.5),
        }
        return min(distances, key=distances.get)

    def set_pen(self, off=0, r=255, g=255, b=255, width=3):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_client.call_async(req)

    def stop_turtle(self):
        self.cmd_pub.publish(Twist())

    def keyboard_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().warn("Clavier non disponible dans ce terminal")
            return

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok() and not self.stop_keyboard:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)

                    if ch == ' ':
                        if self.state == "MANUAL":
                            self.manual_cmd = Twist()
                            self.state = self.prev_auto_state
                            self.get_logger().info("Reprise du mode automatique")
                        else:
                            self.prev_auto_state = self.state
                            self.state = "MANUAL"
                            self.stop_turtle()
                            self.get_logger().info("Mode manuel activé")

                    elif self.state == "MANUAL":
                        cmd = Twist()
                        if ch == 'z':
                            cmd.linear.x = 2.0
                        elif ch == 's':
                            cmd.linear.x = -2.0
                        elif ch == 'q':
                            cmd.angular.z = 2.0
                        elif ch == 'd':
                            cmd.angular.z = -2.0
                        elif ch == 'x':
                            cmd = Twist()

                        self.manual_cmd = cmd
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    @staticmethod
    def distance(p1, p2):
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    @staticmethod
    def copy_pose(p):
        cp = Pose()
        cp.x = p.x
        cp.y = p.y
        cp.theta = p.theta
        cp.linear_velocity = p.linear_velocity
        cp.angular_velocity = p.angular_velocity
        return cp


def main(args=None):
    rclpy.init(args=args)
    node = DrawBoundariesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_turtle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()