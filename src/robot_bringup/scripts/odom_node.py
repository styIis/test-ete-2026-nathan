#!/usr/bin/env python3
"""
Noeud d'odometrie : lit les trames '#O x y th v w' du Nucleo
et publie /odom + le TF odom -> base_link.

Le firmware envoie a 10 Hz apres reception de 'S 1'.
Lecture seule pour l'instant : pas de /cmd_vel.
"""

import math
import serial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().error(f"Ouverture de {port} impossible : {e}")
            raise

        # Le Nucleo redemarre souvent quand le port s'ouvre (DTR).
        self.get_logger().info("Attente du demarrage du Nucleo...")
        self.create_timer(2.0, self._activer_flux)
        self._flux_actif = False

        self.pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Lecture plus rapide que 10 Hz pour ne pas accumuler de retard
        self.create_timer(0.02, self._lire_serie)

        self.trames = 0
        self.rejets = 0
        self.create_timer(5.0, self._diagnostic)

        self.srv_reset = self.create_service(
            Trigger, '~/reset', self._reset_odom)
        
        self.declare_parameter('v_max', 0.5)      # m/s
        self.declare_parameter('w_max', 3.0)       # rad/s
        self.declare_parameter('cmd_timeout', 0.5) # s
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.dernier_cmd = self.get_clock().now()

        self.sub_cmd = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel, 10)
        self.create_timer(0.1, self._envoyer_consigne)   # 10 Hz

    def _cmd_vel(self, msg):
        self.cmd_v = max(-self.v_max, min(self.v_max, msg.linear.x))
        self.cmd_w = max(-self.w_max, min(self.w_max, msg.angular.z))
        self.dernier_cmd = self.get_clock().now()

    def _envoyer_consigne(self):
        age = (self.get_clock().now() - self.dernier_cmd).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self.cmd_v, self.cmd_w
        try:
            self.ser.write(f"C {v:.3f} {w:.3f}\n".encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f"Ecriture serie impossible : {e}")

    def _activer_flux(self):
        if self._flux_actif:
            return
        self.ser.reset_input_buffer()
        self.ser.write(b'S 1\n')
        self._flux_actif = True
        self.get_logger().info("Flux odometrie demande au Nucleo")

    def _lire_serie(self):
        while self.ser.in_waiting:
            try:
                ligne = self.ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().warn(f"Erreur de lecture : {e}")
                return

            if not ligne.startswith('#O '):
                continue

            champs = ligne.split()
            if len(champs) != 6:
                self.rejets += 1
                continue

            try:
                x, y, th, v, w = (float(c) for c in champs[1:])
            except ValueError:
                self.rejets += 1
                continue

            self.trames += 1
            self._publier(x, y, th, v, w)

    def _publier(self, x, y, th, v, w):
        now = self.get_clock().now().to_msg()
        q = yaw_to_quaternion(th)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        # Diagonale : x, y, z, roll, pitch, yaw.
        # Valeurs indicatives, a affiner si le SLAM diverge.
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[35] = 0.05

        self.pub.publish(msg)

    def _diagnostic(self):
        self.get_logger().info(
            f"trames={self.trames}  rejets={self.rejets}")

    def destroy_node(self):
        try:
            self.ser.write(b'C 0 0\n')
            self.ser.write(b'S 0\n')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def _reset_odom(self, request, response):
        self.ser.write(b'R\n')
        self.get_logger().info("Odometrie remise a zero")
        response.success = True
        response.message = "odometrie remise a zero"
        return response

def main():
    rclpy.init()
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()