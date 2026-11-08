#!/usr/bin/env python3
"""
Pont ROS 2 <-> carte moteur ESP32 (firmware carte_moteur_nav.ino).

 - lit les trames '#O x y th v w' (10 Hz apres 'S 1') -> /odom + TF odom -> base_link
 - /cmd_vel (Twist) -> 'C v w' a 10 Hz
 - lit les trames '#F av ar' (flags des Sharp, via le CAN) -> /obstacle_ir (Bool)
 - /actionneur (std_msgs/UInt16, ex. 0x70) -> 'A 70' : la carte moteur relaie
   l'ordre sur le CAN vers la carte actionneur (ventouse / ascenseur)
   ex : ros2 topic pub --once /actionneur std_msgs/msg/UInt16 "{data: 112}"   # 0x70
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
from std_msgs.msg import Bool, UInt16


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_node')

        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Rustine du robot d'ete (moteurs qui ne demarraient pas sous ces vitesses).
        # Sur ce robot la zone morte est compensee dans le firmware : 0 = desactive.
        # Si le robot "cale" a basse vitesse, essayer v_min 0.08 / w_min 0.5.
        self.declare_parameter('v_min', 0.0)    # m/s
        self.declare_parameter('w_min', 0.0)    # rad/s (rotation pure)
        self.v_min = self.get_parameter('v_min').value
        self.w_min = self.get_parameter('w_min').value

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().error(f"Ouverture de {port} impossible : {e}")
            raise

        # L'ESP32 redemarre quand le port s'ouvre (DTR/RTS) : on attend.
        self.get_logger().info("Attente du demarrage de la carte moteur...")
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

        self.pub_ir = self.create_publisher(Bool, 'obstacle_ir', 10)
        self.sub_act = self.create_subscription(
            UInt16, 'actionneur', self._actionneur, 10)

    def _actionneur(self, msg):
        if not 0 < msg.data < 0x800:
            self.get_logger().warn(f"id CAN invalide : {msg.data}")
            return
        self.get_logger().info(f"Ordre actionneur 0x{msg.data:X}")
        try:
            self.ser.write(f"A {msg.data:X}\n".encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f"Ecriture serie impossible : {e}")

    def _cmd_vel(self, msg):
        self.cmd_v = max(-self.v_max, min(self.v_max, msg.linear.x))
        self.cmd_w = max(-self.w_max, min(self.w_max, msg.angular.z))
        self.get_logger().info(f"/cmd_vel recu : {msg.linear.x:.2f} {msg.angular.z:.2f}",
                               throttle_duration_sec=1.0)
        self.dernier_cmd = self.get_clock().now()

    def _envoyer_consigne(self):
        age = (self.get_clock().now() - self.dernier_cmd).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self.cmd_v, self.cmd_w
            if 0.0 < abs(v) < self.v_min:
                v = math.copysign(self.v_min, v)
            if abs(v) < 1e-3 and 0.0 < abs(w) < self.w_min:
                w = math.copysign(self.w_min, w)
        try:
            self.ser.write(f"C {v:.3f} {w:.3f}\n".encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f"Ecriture serie impossible : {e}")

    def _activer_flux(self):
        if self.trames > 0:
            return          # le flux arrive deja
        self.ser.write(b'R\n')     # odometrie a zero a chaque lancement
        self.ser.write(b'S 1\n')
        self.get_logger().info("Odometrie remise a zero, flux demande")

    def _lire_serie(self):
        while self.ser.in_waiting:
            try:
                ligne = self.ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().warn(f"Erreur de lecture : {e}")
                return

            if ligne.startswith('#F '):
                champs = ligne.split()
                if len(champs) >= 2:
                    av = champs[1] == '1'
                    self.pub_ir.publish(Bool(data=av))
                    self.get_logger().info(f"Sharp avant : {'OBSTACLE' if av else 'libre'}")
                continue
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