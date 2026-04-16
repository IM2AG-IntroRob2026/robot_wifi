#!/usr/bin/env python3
"""
Noeud d'exploration — WiFi Mapper (iRobot Create 3)

Scénario d'exécution :
  1. DÉMARRAGE   — attend l'odométrie, calcule la grille de waypoints
  2. EXPLORATION — parcourt les waypoints en boustrophedon (zigzag)
       → ROTATION  vers le prochain waypoint
       → AVANCE    jusqu'au waypoint
       → SCAN      mesure WiFi (déclenche wifi_scanner_node)
  3. ÉVITEMENT   — si obstacle détecté pendant AVANCE :
       → RECUL     0.4 s à -0.10 m/s
       → ROTATION  vers l'extérieur (sens déduit du capteur touché)
       → RETRY     jusqu'à OBSTACLE_SKIP échecs → skip WP
  4. FIN         — publie /exploration_done → heatmap_node génère la carte

Paramètres clés (modifiables via launch) :
  room_width / room_height  — dimensions de la zone à couvrir
  row_spacing               — espacement entre les rangées (0.30 m par défaut)
  room_margin               — marge par rapport aux bords (0.15 m par défaut)
"""

import math
import json
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

try:
    from irobot_create_msgs.msg import HazardDetectionVector
    IROBOT_MSGS = True
except ImportError:
    IROBOT_MSGS = False


class State(Enum):
    WAIT_ODOM    = 0   # attend la première odométrie
    ROTATE       = 1   # tourne vers le prochain waypoint
    MOVE         = 2   # avance vers le waypoint
    SCANNING     = 3   # pause WiFi scan
    ESCAPE_BACK  = 4   # recul après obstacle
    ESCAPE_TURN  = 5   # rotation d'évitement
    DONE         = 6   # exploration terminée


class ExplorerNode(Node):

    ESCAPE_BACK_DUR = 0.40   # secondes de recul
    ESCAPE_TURN_DUR = 0.80   # secondes de rotation d'évitement
    OBSTACLE_SKIP   = 4      # skip le WP après N échecs consécutifs

    def __init__(self):
        super().__init__('explorer_node')

        # ── Paramètres ────────────────────────────────────────────────
        self.declare_parameter('room_width',         2.0)   # m
        self.declare_parameter('room_height',        2.0)   # m
        self.declare_parameter('row_spacing',        0.30)  # m  ← réduit pour petites salles
        self.declare_parameter('room_margin',        0.15)  # m  ← marge bord de salle
        self.declare_parameter('linear_speed',       0.12)  # m/s
        self.declare_parameter('angular_speed',      0.45)  # rad/s
        self.declare_parameter('waypoint_tolerance', 0.12)  # m
        self.declare_parameter('scan_duration',      2.0)   # s

        self.room_w   = self.get_parameter('room_width').value
        self.room_h   = self.get_parameter('room_height').value
        self.row_sp   = self.get_parameter('row_spacing').value
        self.margin   = self.get_parameter('room_margin').value
        self.v_lin    = self.get_parameter('linear_speed').value
        self.v_ang    = self.get_parameter('angular_speed').value
        self.tol      = self.get_parameter('waypoint_tolerance').value
        self.scan_dt  = self.get_parameter('scan_duration').value

        # ── Publishers ────────────────────────────────────────────────
        self.cmd_pub     = self.create_publisher(Twist,  'cmd_vel',          10)
        self.trigger_pub = self.create_publisher(Bool,   'trigger_scan',     10)
        self.done_pub    = self.create_publisher(Bool,   'exploration_done', 10)
        self.pose_pub    = self.create_publisher(String, 'robot_pose_json',  10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Odometry, 'odom', self._odom_cb,
                                 qos_profile_sensor_data)
        if IROBOT_MSGS:
            self.create_subscription(
                HazardDetectionVector, 'hazard_detection',
                self._hazard_cb, qos_profile_sensor_data)
        else:
            self.get_logger().warn(
                'irobot_create_msgs non disponible — détection obstacles désactivée')

        # ── État interne ──────────────────────────────────────────────
        self.x = self.y = self.yaw = 0.0
        self.odom_ok  = False
        self.origin_x = self.origin_y = 0.0

        self.state    = State.WAIT_ODOM
        self.waypoints: list[tuple[float, float]] = []
        self.wp_idx   = 0
        self.scan_t0  = None
        self.start_t  = None          # heure de début exploration

        # Obstacle
        self.obstacle       = False
        self.hazard_frame   = ''      # frame_id du dernier capteur touché
        self.escape_t0      = None    # chrono phase recul/rotation
        self.escape_turn_dir = 1.0   # +1 = gauche, -1 = droite
        self.obstacle_n     = 0      # tentatives échouées sur le WP courant

        self.create_timer(0.05, self._loop)

    # ── Callbacks ────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not self.odom_ok:
            self.odom_ok  = True
            self.origin_x = self.x
            self.origin_y = self.y

    def _hazard_cb(self, msg):
        for h in msg.detections:
            if h.type in (1, 2):   # BUMP=1  CLIFF=2
                if not self.obstacle:
                    self.hazard_frame = h.header.frame_id
                self.obstacle = True

    # ── Helpers ──────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _wrap(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _elapsed(self, t0) -> float:
        return (self.get_clock().now() - t0).nanoseconds / 1e9

    def _gen_waypoints(self) -> list[tuple[float, float]]:
        """
        Grille boustrophedon avec marge sur les bords.
        Les waypoints restent à l'intérieur de la zone utile.
        """
        w = self.room_w - 2 * self.margin
        h = self.room_h - 2 * self.margin
        if w <= 0 or h <= 0:
            self.get_logger().error('room_margin trop grand par rapport à room_width/height !')
            w = max(0.3, w)
            h = max(0.3, h)

        n_cols = max(2, round(w / self.row_sp) + 1)
        n_rows = max(2, round(h / self.row_sp) + 1)

        # Espacement réel recalculé pour couvrir exactement la zone utile
        step_x = w / max(1, n_cols - 1)
        step_y = h / max(1, n_rows - 1)

        ox = self.origin_x + self.margin
        oy = self.origin_y + self.margin

        pts: list[tuple[float, float]] = []
        for r in range(n_rows):
            row = [(ox + c * step_x, oy + r * step_y) for c in range(n_cols)]
            if r % 2 == 1:
                row.reverse()   # zigzag
            pts.extend(row)

        sep = '─' * 52
        self.get_logger().info(sep)
        self.get_logger().info('  DÉMARRAGE EXPLORATION WiFi')
        self.get_logger().info(sep)
        self.get_logger().info(
            f'  Salle utile : {w:.2f} m × {h:.2f} m '
            f'(marge {self.margin} m)')
        self.get_logger().info(
            f'  Grille      : {n_cols} col × {n_rows} lig '
            f'= {len(pts)} waypoints')
        self.get_logger().info(
            f'  Espacement  : {step_x:.2f} m (X)  {step_y:.2f} m (Y)')
        self.get_logger().info(
            f'  Durée scan  : {self.scan_dt} s/point')
        eta = len(pts) * (self.scan_dt + self.row_sp / max(0.01, self.v_lin))
        self.get_logger().info(
            f'  Durée est.  : ~{int(eta/60)} min {int(eta%60)} s')
        self.get_logger().info(sep)
        return pts

    def _escape_direction(self) -> float:
        """
        Détermine le sens de rotation d'évitement (+1=gauche, -1=droite)
        à partir du capteur qui a détecté l'obstacle.
        """
        frame = self.hazard_frame.lower()
        if 'right' in frame:
            return 1.0    # obstacle à droite → tourne à gauche
        if 'left' in frame:
            return -1.0   # obstacle à gauche → tourne à droite
        # Centre ou inconnu : tourne du côté opposé au waypoint
        if self.wp_idx < len(self.waypoints):
            tx, ty = self.waypoints[self.wp_idx]
            err = self._wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)
            return -math.copysign(1.0, err)   # s'écarter de la direction du WP
        return 1.0

    def _pub_pose(self):
        msg = String()
        msg.data = json.dumps({'x': round(self.x, 3), 'y': round(self.y, 3)})
        self.pose_pub.publish(msg)

    def _progress_str(self) -> str:
        pct = 100 * self.wp_idx // max(1, len(self.waypoints))
        bar = '█' * (pct // 5) + '░' * (20 - pct // 5)
        return f'[{bar}] {pct:3d}%  WP {self.wp_idx+1}/{len(self.waypoints)}'

    # ── Boucle de contrôle (50 Hz) ───────────────────────────────────

    def _loop(self):

        # ══ 0. Attente odométrie ══════════════════════════════════════
        if self.state == State.WAIT_ODOM:
            if self.odom_ok:
                self.waypoints = self._gen_waypoints()
                self.start_t   = self.get_clock().now()
                self.state     = State.ROTATE
            return

        # ══ 6. Terminé ════════════════════════════════════════════════
        if self.state == State.DONE:
            self._stop()
            return

        # ══ 4. Recul après obstacle ═══════════════════════════════════
        if self.state == State.ESCAPE_BACK:
            if self._elapsed(self.escape_t0) < self.ESCAPE_BACK_DUR:
                t = Twist()
                t.linear.x = -0.10
                self.cmd_pub.publish(t)
            else:
                self._stop()
                # Prépare la rotation d'évitement
                self.escape_turn_dir = self._escape_direction()
                self.escape_t0 = self.get_clock().now()
                self.state = State.ESCAPE_TURN
            return

        # ══ 5. Rotation d'évitement ═══════════════════════════════════
        if self.state == State.ESCAPE_TURN:
            if self._elapsed(self.escape_t0) < self.ESCAPE_TURN_DUR:
                t = Twist()
                t.angular.z = self.escape_turn_dir * self.v_ang
                self.cmd_pub.publish(t)
            else:
                self._stop()
                self.obstacle = False
                self.obstacle_n += 1

                if self.obstacle_n >= self.OBSTACLE_SKIP:
                    self.get_logger().warn(
                        f'  ⚠ WP {self.wp_idx+1} inaccessible '
                        f'({self.obstacle_n} tentatives) → ignoré')
                    self.obstacle_n = 0
                    self.wp_idx += 1
                else:
                    self.get_logger().info(
                        f'  ↺ Évitement #{self.obstacle_n} '
                        f'({self.hazard_frame or "inconnu"}) → retry WP {self.wp_idx+1}')
                self.state = State.ROTATE
            return

        # ══ Fin des waypoints ? ═══════════════════════════════════════
        if self.wp_idx >= len(self.waypoints):
            self._stop()
            self.state = State.DONE
            elapsed = self._elapsed(self.start_t) if self.start_t else 0
            sep = '─' * 52
            self.get_logger().info(sep)
            self.get_logger().info('  EXPLORATION TERMINÉE')
            self.get_logger().info(
                f'  {len(self.waypoints)} points | '
                f'durée : {int(elapsed//60)} min {int(elapsed%60)} s')
            self.get_logger().info('  Génération de la carte en cours...')
            self.get_logger().info(sep)
            msg = Bool(); msg.data = True
            self.done_pub.publish(msg)
            return

        # ══ Obstacle détecté pendant déplacement ═════════════════════
        if self.obstacle and self.state not in (
                State.SCANNING, State.ESCAPE_BACK, State.ESCAPE_TURN):
            self._stop()
            self.escape_t0 = self.get_clock().now()
            self.state = State.ESCAPE_BACK
            self.get_logger().warn(
                f'  ✖ Obstacle [{self.hazard_frame or "?"}] — recul + évitement')
            return

        tx, ty = self.waypoints[self.wp_idx]
        dx, dy = tx - self.x, ty - self.y
        dist   = math.hypot(dx, dy)

        # ══ 3. Arrivé au waypoint → scan ═════════════════════════════
        if dist < self.tol:
            self.obstacle_n = 0
            if self.state != State.SCANNING:
                self._stop()
                self.state   = State.SCANNING
                self.scan_t0 = self.get_clock().now()
                self._pub_pose()
                trig = Bool(); trig.data = True
                self.trigger_pub.publish(trig)
                self.get_logger().info(
                    f'  ◉ {self._progress_str()} '
                    f'| scan ({self.x:.2f}, {self.y:.2f})')
            else:
                if self._elapsed(self.scan_t0) >= self.scan_dt:
                    self.wp_idx += 1
                    self.state = State.ROTATE
            return

        # ══ 1. Rotation vers le waypoint ═════════════════════════════
        if self.state == State.ROTATE:
            target_yaw = math.atan2(dy, dx)
            err = self._wrap(target_yaw - self.yaw)
            if abs(err) < 0.04:
                self.state = State.MOVE
                self.get_logger().info(
                    f'  → {self._progress_str()} '
                    f'| vers ({tx:.2f}, {ty:.2f})  dist={dist:.2f}m')
            else:
                t = Twist()
                speed = self.v_ang * min(1.0, abs(err) / 0.4)
                t.angular.z = math.copysign(max(0.08, speed), err)
                self.cmd_pub.publish(t)
            return

        # ══ 2. Avance vers le waypoint ════════════════════════════════
        if self.state == State.MOVE:
            target_yaw  = math.atan2(dy, dx)
            heading_err = self._wrap(target_yaw - self.yaw)
            if abs(heading_err) > 0.25:
                self.state = State.ROTATE
                return
            t = Twist()
            t.linear.x  = max(0.05, min(self.v_lin, dist * 0.6))
            t.angular.z = heading_err * 1.8
            self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
