#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose


class OpponentDetector(Node):
    def __init__(self):
        super().__init__('opponent_detector')

        self.declare_parameter('angle_offset', 0.0)   # la rotation est portée par la TF
        self.declare_parameter('range_max', 4.0)        # m, range_max=25 du driver est faux
        self.declare_parameter('range_min', 0.10)       # m, ignore le châssis
        self.declare_parameter('cluster_gap', 0.15)     # m, séparation entre 2 clusters
        self.declare_parameter('min_points', 3)
        self.declare_parameter('min_width', 0.05)       # m
        self.declare_parameter('max_width', 0.60)       # m

        self.offset = math.radians(
            self.get_parameter('angle_offset').value)

        self.pub = self.create_publisher(PoseArray, '/opponents', 10)
        self.create_subscription(LaserScan, '/scan_high', self.cb, 10)

        self.get_logger().info(
            f'opponent_detector actif (offset={math.degrees(self.offset):.1f}°)')

    def cb(self, msg: LaserScan):
        rmin = self.get_parameter('range_min').value
        rmax = self.get_parameter('range_max').value
        gap = self.get_parameter('cluster_gap').value
        nmin = self.get_parameter('min_points').value
        wmin = self.get_parameter('min_width').value
        wmax = self.get_parameter('max_width').value

        # 1. Points valides -> cartésien (repère robot), en gardant l'indice
        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            a = msg.angle_min + i * msg.angle_increment + self.offset
            pts.append((i, r * math.cos(a), r * math.sin(a)))

        if not pts:
            self.pub.publish(PoseArray(header=msg.header))
            return

        # 2. Clustering par proximité spatiale
        clusters, cur = [], [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[1] - cur[-1][1], p[2] - cur[-1][2]) < gap:
                cur.append(p)
            else:
                clusters.append(cur)
                cur = [p]
        clusters.append(cur)

        # 2b. BOUCLAGE : le scan est un cercle, pas un segment.
        #     Si le premier et le dernier cluster se touchent, ils n'en font qu'un.
        if len(clusters) > 1:
            a_end = clusters[-1][-1]
            b_start = clusters[0][0]
            contigu = (len(msg.ranges) - 1 - a_end[0] + b_start[0]) <= 2
            proche = math.hypot(b_start[1] - a_end[1],
                                b_start[2] - a_end[2]) < gap
            if contigu and proche:
                clusters[0] = clusters[-1] + clusters[0]
                clusters.pop()

        # 3. Filtre par taille
        out = PoseArray()
        out.header = msg.header
        for c in clusters:
            if len(c) < nmin:
                continue
            xs = [p[1] for p in c]
            ys = [p[2] for p in c]
            w = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            if not (wmin <= w <= wmax):
                continue
            pose = Pose()
            pose.position.x = sum(xs) / len(c)
            pose.position.y = sum(ys) / len(c)
            pose.orientation.w = 1.0
            out.poses.append(pose)

        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(OpponentDetector())
    rclpy.shutdown()


if __name__ == '__main__':
    main()