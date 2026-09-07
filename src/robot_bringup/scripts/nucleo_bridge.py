#!/usr/bin/env python3
# ============================================================
#  Pont ROS2 -> Nucleo  (affichage direction obstacle)
#  Projet robotique IUT GEII
#
#  Role :
#    - SOUSCRIT au topic /obstacle_direction (std_msgs/String)
#    - traduit la direction en 1 octet et l'ecrit sur l'UART du Nucleo
#
#  Protocole (1 octet) :
#    'F'=FRONT  'L'=LEFT  'R'=RIGHT  'B'=BACK  'C'=CLEAR
#
#  Lancement :  ros2 run robot_bringup nucleo_bridge.py
# ============================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

# --- A ADAPTER : verifie avec  ls /dev/ttyACM*  ---
PORT_NUCLEO = '/dev/nucleo'
BAUD = 115200

CODES = {
    'FRONT': b'F',
    'LEFT':  b'L',
    'RIGHT': b'R',
    'BACK':  b'B',
    'CLEAR': b'C',
}

class NucleoBridge(Node):
    def __init__(self):
        super().__init__('nucleo_bridge')
        self.ser = serial.Serial(PORT_NUCLEO, BAUD, timeout=0.1)
        self.last_code = b'C'          # defaut : rien affiche
        self.create_subscription(String, '/obstacle_direction', self.on_direction, 10)
        # heartbeat : renvoie l'etat courant 2x/s (re-sync si le Nucleo reboote)
        self.create_timer(0.5, self.heartbeat)
        self.get_logger().info(f'Pont Nucleo demarre sur {PORT_NUCLEO}')

    def on_direction(self, msg):
        code = CODES.get(msg.data)
        if code is None:
            self.get_logger().warn(f'direction inconnue : {msg.data!r}')
            return
        self.last_code = code
        self.ser.write(code)           # ecriture immediate -> faible latence
        self.get_logger().info(f'-> Nucleo : {msg.data}')

    def heartbeat(self):
        self.ser.write(self.last_code) # re-envoi periodique de l'etat courant

def main():
    rclpy.init()
    node = NucleoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()