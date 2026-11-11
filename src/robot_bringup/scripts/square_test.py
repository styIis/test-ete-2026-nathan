#!/usr/bin/env python3
"""square_test — fait parcourir au robot un carré de côté `side` et le ramène au départ,
asservi sur /odom. But : mesurer au mètre l'écart réel à l'arrivée = erreur d'odométrie.

Modes :
  strafe (défaut) : translation pure, le robot garde son cap (avant, gauche, arrière, droite).
                    Teste le diamètre de roue, les angles des roues et le glissement.
  turn            : avance puis tourne de 90° à chaque coin, comme un robot différentiel.
                    Teste en plus la rotation (WHEEL_DIST).

Le carré part vers l'avant puis vers la gauche (sens trigo).
Fermer teleop avant de lancer : deux publishers sur /cmd_vel se contredisent.

  ros2 run robot_bringup square_test.py
  ros2 run robot_bringup square_test.py --ros-args -p mode:=turn -p side:=1.0 -p speed:=0.15
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SquareTest(Node):
    def __init__(self):
        super().__init__('square_test')
        self.declare_parameter('side', 0.5)          # m
        self.declare_parameter('mode', 'strafe')     # strafe | turn
        self.declare_parameter('speed', 0.15)        # m/s max en translation
        self.declare_parameter('w_max', 0.6)         # rad/s max en rotation
        self.declare_parameter('kp_lin', 1.5)
        self.declare_parameter('kp_ang', 2.0)
        self.declare_parameter('pos_tol', 0.005)     # m
        self.declare_parameter('ang_tol', 0.01)      # rad (~0,6°)
        self.declare_parameter('segment_timeout', 30.0)  # s, sécurité par segment

        p = lambda n: self.get_parameter(n).value
        self.side, self.mode = p('side'), p('mode')
        self.speed, self.w_max = p('speed'), p('w_max')
        self.kp_lin, self.kp_ang = p('kp_lin'), p('kp_ang')
        self.pos_tol, self.ang_tol = p('pos_tol'), p('ang_tol')
        self.seg_timeout = p('segment_timeout')
        if self.mode not in ('strafe', 'turn'):
            raise ValueError("mode doit valoir 'strafe' ou 'turn'")

        self.pose = None        # (x, y, th) odom courante
        self.start = None       # pose de départ
        self.segments = []      # [(x, y, cap)] dans le repère odom
        self.seg = 0
        self.phase = 'rotate'
        self.seg_t0 = None
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.05, self.loop)
        self.get_logger().info('Attente de /odom...')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def build_segments(self):
        x0, y0, th0 = self.start
        s = self.side
        # Coins du carré dans le repère robot au départ : avant, puis gauche...
        local = [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]
        c, si = math.cos(th0), math.sin(th0)
        for k, (lx, ly) in enumerate(local):
            heading = th0 + (k * math.pi / 2 if self.mode == 'turn' else 0.0)
            self.segments.append((x0 + c * lx - si * ly, y0 + si * lx + c * ly, wrap(heading)))
        # En mode turn, on finit par un dernier quart de tour pour retrouver le cap initial
        if self.mode == 'turn':
            self.segments.append((x0, y0, wrap(th0)))

    def publish(self, vx=0.0, vy=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(w)
        self.cmd_pub.publish(t)

    def loop(self):
        if self.done:
            self.publish()
            return
        if self.pose is None:
            return
        if self.start is None:
            self.start = self.pose
            self.build_segments()
            self.seg_t0 = self.get_clock().now()
            self.get_logger().info(
                f'Depart : x={self.start[0]:.3f} y={self.start[1]:.3f} '
                f'th={math.degrees(self.start[2]):.1f} deg, mode {self.mode}, cote {self.side} m')

        x, y, th = self.pose
        tx, ty, tth = self.segments[self.seg]
        e_th = wrap(tth - th)
        w = max(-self.w_max, min(self.w_max, self.kp_ang * e_th))

        elapsed = (self.get_clock().now() - self.seg_t0).nanoseconds * 1e-9
        if elapsed > self.seg_timeout:
            self.get_logger().error(f'Segment {self.seg + 1} : timeout, arret.')
            self.finish()
            return

        if self.phase == 'rotate':
            # On s'oriente d'abord (immédiat en mode strafe : le cap ne change pas)
            if abs(e_th) < self.ang_tol:
                self.phase = 'move'
            self.publish(w=w)
            return

        # Phase move : erreur de position exprimée dans le repère robot + maintien du cap
        ex, ey = tx - x, ty - y
        dist = math.hypot(ex, ey)
        if dist < self.pos_tol and abs(e_th) < self.ang_tol:
            self.get_logger().info(
                f'Coin {self.seg + 1}/{len(self.segments)} atteint '
                f'(odom x={x:.3f} y={y:.3f} th={math.degrees(th):.1f} deg)')
            self.seg += 1
            self.phase = 'rotate'
            self.seg_t0 = self.get_clock().now()
            if self.seg >= len(self.segments):
                self.finish()
            else:
                self.publish()
            return

        erx = math.cos(th) * ex + math.sin(th) * ey
        ery = -math.sin(th) * ex + math.cos(th) * ey
        v = self.kp_lin * dist
        scale = min(v, self.speed) / dist if dist > 1e-6 else 0.0
        self.publish(erx * scale, ery * scale, w)

    def finish(self):
        self.done = True
        self.publish()
        x, y, th = self.pose
        x0, y0, th0 = self.start
        self.get_logger().info(
            'Termine. Selon l\'odometrie, ecart au depart : '
            f'dx={1000 * (x - x0):.1f} mm, dy={1000 * (y - y0):.1f} mm, '
            f'dth={math.degrees(wrap(th - th0)):.2f} deg. '
            'Mesure maintenant l\'ecart REEL au metre : la difference = erreur d\'odometrie.')


def main():
    rclpy.init()
    node = SquareTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
