#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# --- Tuning parameters ---
THRESHOLD_M  = 0.30    # alert distance in metres (>= 0.15 m sensor floor)
ANGLE_OFFSET = -151.0     # degrees; set after calibration so cable side = BACK
ANGLE_OFFSET_LD06 = 0
CALIBRATE    = False   # True: only print raw angle of nearest point

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(String, '/obstacle_direction', 10)
        self.last = None
        self.tick = 0

    def scan_cb(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        physical = (np.isfinite(ranges)
                    & (ranges >= msg.range_min)
                    & (ranges <= msg.range_max))

        # --- calibration mode: report raw angle of nearest point ---
        if CALIBRATE:
            if np.any(physical):
                i = int(np.argmin(np.where(physical, ranges, np.inf)))
                deg = math.degrees(angles[i]) % 360.0
                self.tick += 1
                if self.tick % 6 == 0:      # ~2 Hz log
                    self.get_logger().info(
                        f'nearest raw angle = {deg:6.1f} deg  '
                        f'({ranges[i]*100:.0f} cm)  ->  '
                        f'set ANGLE_OFFSET = {(180.0 - deg):.1f}')
            return

        # --- normal mode ---
        valid = physical & (ranges <= THRESHOLD_M)
        if not np.any(valid):
            self.emit('CLEAR')
            return
        i = int(np.argmin(np.where(valid, ranges, np.inf)))
        self.emit(self.classify(angles[i]), dist=float(ranges[i]))

    def classify(self, angle_rad):
        deg = (math.degrees(angle_rad) + ANGLE_OFFSET) % 360.0
        if deg < 45 or deg >= 315:
            return 'FRONT'
        elif deg < 135:
            return 'LEFT'
        elif deg < 225:
            return 'BACK'
        else:
            return 'RIGHT'

    def emit(self, direction, dist=None):
        if direction == self.last:
            return
        self.last = direction
        m = String(); m.data = direction
        self.pub.publish(m)
        d = f' ({dist*100:.0f} cm)' if dist is not None else ''
        self.get_logger().info(f'{direction}{d}')

def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()