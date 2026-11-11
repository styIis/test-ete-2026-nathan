#!/usr/bin/env python3
"""holo_bridge — pont série ROS 2 <-> ESP32 de la base holonome (firmware holo_base.ino).

  /cmd_vel (Twist)  ->  "C vx vy w"   renvoyé à cmd_rate Hz (nourrit le watchdog de l'ESP32)
  "#O x y th vx vy w"  ->  /odom (Odometry) + TF odom -> base_link
  service ~/reset (std_srvs/Empty)  ->  "R" (odométrie à zéro)

Si aucun /cmd_vel n'arrive pendant cmd_timeout secondes, on envoie une consigne nulle :
avec teleop_twist_keyboard, le robot avance tant que la touche est maintenue.
"""
import math
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


class HoloBridge(Node):
    def __init__(self):
        super().__init__('holo_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_timeout', 0.7)   # s ; > délai d'autorépétition du clavier
        self.declare_parameter('cmd_rate', 20.0)     # Hz
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        cmd_rate = self.get_parameter('cmd_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # DTR/RTS pilotent EN et IO0 de l'ESP32 (circuit d'auto-reset de la DevKit).
        # On les force à l'état inactif AVANT l'ouverture : pas de reset ni de blocage en boot.
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.get_logger().info(f'Port {port} ouvert')
        time.sleep(0.5)
        self.ser.reset_input_buffer()

        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time = None
        self.rx_buf = b''
        self.got_odom = False
        self.create_timer(5.0, self.check_link)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = (TransformBroadcaster(self)
                               if self.get_parameter('publish_tf').value else None)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_service(Empty, '~/reset', self.on_reset)
        self.create_timer(1.0 / cmd_rate, self.send_cmd)
        self.create_timer(0.01, self.read_serial)

    def check_link(self):
        if not self.got_odom:
            self.get_logger().warn('Aucune trame #O recue : mauvais port, firmware absent, '
                                   'ou port deja ouvert ailleurs ?')

    # ---------- envoi ----------
    def on_cmd_vel(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def send_cmd(self):
        vx, vy, w = self.cmd
        if (self.last_cmd_time is None or
                (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout):
            vx = vy = w = 0.0
        line = f'C {vx:.3f} {vy:.3f} {w:.3f}\n'
        if vx or vy or w:
            self.get_logger().info(f'-> {line.strip()}', throttle_duration_sec=1.0)
        self.write(line)

    def on_reset(self, request, response):
        self.write('R\n')
        return response

    def write(self, text: str):
        try:
            self.ser.write(text.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Ecriture serie impossible : {e}', throttle_duration_sec=2.0)

    # ---------- réception ----------
    def read_serial(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except serial.SerialException as e:
            self.get_logger().error(f'Lecture serie impossible : {e}', throttle_duration_sec=2.0)
            return
        if not data:
            return
        self.rx_buf += data
        while b'\n' in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split(b'\n', 1)
            self.handle_line(line.decode(errors='ignore').strip())
        if len(self.rx_buf) > 512:   # pas de '\n' depuis longtemps : on purge
            self.rx_buf = b''

    def handle_line(self, line: str):
        if line.startswith('#O'):
            parts = line.split()
            if len(parts) != 7:
                return
            try:
                x, y, th, vx, vy, w = map(float, parts[1:])
            except ValueError:
                return
            if not self.got_odom:
                self.got_odom = True
                self.get_logger().info('Premiere trame #O recue : liaison ESP32 OK')
            self.publish_odom(x, y, th, vx, vy, w)
        elif line.startswith('#E'):
            self.get_logger().error(f'ESP32 : {line}')
        elif line.startswith('#'):
            self.get_logger().info(f'ESP32 : {line}')

    def publish_odom(self, x, y, th, vx, vy, w):
        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(th / 2.0), math.cos(th / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = HoloBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write('C 0 0 0\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
