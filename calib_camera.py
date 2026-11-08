#!/usr/bin/env python3
"""
Calibration de la camera SANS interface graphique (Pi en SSH).

Usage (camera deja lancee sur /camera/image_raw) :
    python3 calib_camera.py --cols 9 --rows 6 --square 0.025

  --cols / --rows : nombre de coins INTERIEURS du damier (pas de cases !)
                    damier OpenCV classique 10 x 7 cases -> 9 x 6 coins
  --square        : cote d'une case en METRES, mesure a la regle sur l'impression

Deroulement : bouger lentement le damier devant la camera (pres, loin, a
gauche, a droite, en haut, en bas, incline). Une image est gardee des que le
damier est vu ET qu'il a assez bouge depuis la precedente. Apres N images,
le calcul se lance et le fichier est ecrit la ou v4l2_camera le cherche :
    ~/.ros/camera_info/hd_pro_webcam_c920.yaml
Relancer ensuite la camera : /camera/camera_info contient la calibration.
"""
import argparse
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Calib(Node):
    def __init__(self, a):
        super().__init__('calib_camera')
        self.a = a
        self.taille = (a.cols, a.rows)
        self.bridge = CvBridge()
        self.pts_img = []
        self.dernier = None
        self.prec = None
        self.dim = None
        self.fini = False
        # coins du damier dans son propre repere (z = 0)
        self.objp = np.zeros((a.cols * a.rows, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
        self.create_subscription(Image, a.topic, self.cb, 1)
        self.get_logger().info(
            f"Damier {a.cols}x{a.rows} coins, case {a.square*1000:.1f} mm. "
            f"Objectif : {a.n} images. Bouge le damier lentement.")

    def cb(self, msg):
        if self.fini:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        self.dim = (img.shape[1], img.shape[0])
        ok, coins = cv2.findChessboardCorners(
            img, self.taille, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        if not ok:
            self.prec = None
            return
        centre = coins.reshape(-1, 2).mean(axis=0)
        etendue = np.ptp(coins.reshape(-1, 2), axis=0)
        # on n'accepte que des images NETTES : damier immobile depuis l'image precedente
        immobile = self.prec is not None and np.linalg.norm(centre - self.prec) < 1.5
        self.prec = centre
        if not immobile:
            return
        # on ne garde l'image que si le damier a assez bouge (position ou taille)
        if self.dernier is not None:
            c0, e0 = self.dernier
            if np.linalg.norm(centre - c0) < 40 and np.linalg.norm(etendue - e0) < 25:
                return
        coins = cv2.cornerSubPix(img, coins, (11, 11), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        self.pts_img.append(coins)
        self.dernier = (centre, etendue)
        self.get_logger().info(f"image {len(self.pts_img)}/{self.a.n} gardee")
        if len(self.pts_img) >= self.a.n:
            self.fini = True
            self.calculer()

    def calculer(self):
        self.get_logger().info("Calcul de la calibration...")
        objpts = [self.objp] * len(self.pts_img)
        w, h = self.dim
        # Point de depart realiste (C920 en 640x480 : f ~ 620 px) + pixels carres :
        # evite que le calcul derive vers une focale absurde si les vues sont peu variees.
        K0 = np.array([[620.0, 0, w / 2], [0, 620.0, h / 2], [0, 0, 1]])
        flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO
        err, K, D, _, _ = cv2.calibrateCamera(objpts, self.pts_img, self.dim, K0, None, flags=flags)
        self.get_logger().info(f"Erreur de reprojection : {err:.3f} px (bon si < 0.5, acceptable < 1)")
        if not (400 < K[0, 0] < 900) or abs(K[0, 2] - w / 2) > 60 or abs(K[1, 2] - h / 2) > 60:
            self.get_logger().error(
                f"Resultat INCOHERENT (fx={K[0,0]:.0f}, cx={K[0,2]:.0f}, cy={K[1,2]:.0f}) : fichier NON ecrit. "
                "Recommence en INCLINANT franchement le damier (30-45 deg) dans tous les sens "
                "et en variant la distance.")
            return
        d = D.flatten()[:5].tolist()
        P = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        fmt = lambda l: '[' + ', '.join(f"{v:.6f}" for v in l) + ']'
        yaml = (
            f"image_width: {w}\n"
            f"image_height: {h}\n"
            f"camera_name: {self.a.name}\n"
            f"camera_matrix:\n  rows: 3\n  cols: 3\n  data: {fmt(K.flatten().tolist())}\n"
            f"distortion_model: plumb_bob\n"
            f"distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: {fmt(d)}\n"
            f"rectification_matrix:\n  rows: 3\n  cols: 3\n  data: {fmt([1,0,0,0,1,0,0,0,1])}\n"
            f"projection_matrix:\n  rows: 3\n  cols: 4\n  data: {fmt(P)}\n")
        chemin = os.path.expanduser(f"~/.ros/camera_info/{self.a.name}.yaml")
        os.makedirs(os.path.dirname(chemin), exist_ok=True)
        with open(chemin, 'w') as f:
            f.write(yaml)
        self.get_logger().info(f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
        self.get_logger().info(f"Ecrit : {chemin}  -> relancer la camera. Ctrl+C pour quitter.")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cols', type=int, default=9)
    p.add_argument('--rows', type=int, default=6)
    p.add_argument('--square', type=float, default=0.025)
    p.add_argument('--n', type=int, default=25)
    p.add_argument('--topic', default='/camera/image_raw')
    p.add_argument('--name', default='hd_pro_webcam_c920')
    a = p.parse_args()
    rclpy.init()
    node = Calib(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
