#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

ID_BLEU, ID_JAUNE, ID_VIDE = 36, 47, 41
NOMS = {ID_BLEU: 'bleu', ID_JAUNE: 'jaune', ID_VIDE: 'vide'}

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        if hasattr(aruco, 'ArucoDetector'):
            self.dico = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            self.detector = aruco.ArucoDetector(self.dico, aruco.DetectorParameters())
        else:
            self.dico = aruco.Dictionary_get(aruco.DICT_4X4_50)
            self.params = aruco.DetectorParameters_create()
            self.detector = None
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.cb, 10)
        self.pub = self.create_publisher(String, '/bricks/detected', 10)

    def cb(self, msg):
        self.get_logger().info('image reçue', throttle_duration_sec=2.0)
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            coins, ids, _ = self.detector.detectMarkers(gris)
        else:
            coins, ids, _ = aruco.detectMarkers(gris, self.dico, parameters=self.params)
        if ids is None:
            return
        # (id, x du centre) trié de gauche à droite
        vus = sorted(
            ((int(i[0]), float(c[0][:, 0].mean())) for i, c in zip(ids, coins)),
            key=lambda t: t[1])
        txt = ' | '.join(f"{NOMS.get(i, i)}@{int(x)}" for i, x in vus)
        self.get_logger().info(txt)
        self.pub.publish(String(data=txt))

def main():
    rclpy.init()
    rclpy.spin(ArucoDetector())

if __name__ == '__main__':
    main()