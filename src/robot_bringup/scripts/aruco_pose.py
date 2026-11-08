#!/usr/bin/env python3
"""
aruco_pose.py -- position 3D du tag ArUco de la brique.

Entrees :
  /camera/image_raw, /camera/camera_info (calibration), TF (map, base_link, camera_optical)
Sorties :
  /brique/tag_base  (PoseStamped, repere base_link) : ou est le tag par rapport au robot
                    -> sert a l'approche finale (distance devant, decalage lateral, angle)
  /brique/pose      (PoseStamped, repere map) : centre de la brique au sol, orientation =
                    cap que le robot doit avoir pour lui faire FACE (-> but Nav2)
  /aruco/debug      (Image, 2 Hz) : image avec le tag et ses axes dessines (Foxglove)
Log : distance et angle du tag, 2 fois par seconde.

Hypotheses : tag colle au centre de la grande face de la brique, tourne vers le robot.
"""
import math

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (enregistre la conversion PoseStamped pour tf2)
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def quat_de_matrice(R):
    """Matrice de rotation 3x3 -> quaternion (x, y, z, w)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def matrice_de_quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ArucoPose(Node):
    def __init__(self):
        super().__init__('aruco_pose')
        self.declare_parameter('tag_id', 2)
        self.declare_parameter('tag_size', 0.07)          # cote de la partie NOIRE (m)
        self.declare_parameter('demi_epaisseur', 0.055)   # brique de 11 cm : tag -> centre
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.tag_id = self.get_parameter('tag_id').value
        s = self.get_parameter('tag_size').value
        self.demi_ep = self.get_parameter('demi_epaisseur').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # coins du tag dans son repere (ordre ArUco : haut-gauche, haut-droit, bas-droit, bas-gauche)
        self.obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0],
                             [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], dtype=np.float32)

        if hasattr(aruco, 'ArucoDetector'):
            dico = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            prm = aruco.DetectorParameters()
            prm.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX  # coins plus precis
            self.detector = aruco.ArucoDetector(dico, prm)
            self.detect = lambda g: self.detector.detectMarkers(g)
        else:
            dico = aruco.Dictionary_get(aruco.DICT_4X4_50)
            prm = aruco.DetectorParameters_create()
            prm.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self.detect = lambda g: aruco.detectMarkers(g, dico, parameters=prm)

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/camera/camera_info', self.cb_info, 1)
        self.create_subscription(Image, '/camera/image_raw', self.cb_image, 1)
        self.pub_base = self.create_publisher(PoseStamped, '/brique/tag_base', 10)
        self.pub_map = self.create_publisher(PoseStamped, '/brique/pose', 10)
        self.pub_dbg = self.create_publisher(Image, '/aruco/debug', 1)
        self.t_dbg = self.get_clock().now()
        self.get_logger().info(f"Recherche du tag {self.tag_id} ({s * 100:.1f} cm)")

    def cb_info(self, msg):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] == 0:
            self.get_logger().warn("camera_info sans calibration !", throttle_duration_sec=10.0)
            return
        self.K = K
        self.D = np.array(msg.d, dtype=np.float64)

    def transforme(self, ps, cible):
        """PoseStamped -> repere cible, a l'instant de l'image si possible, sinon le plus recent."""
        try:
            return self.tf_buffer.transform(ps, cible, timeout=Duration(seconds=0.05))
        except Exception:
            ps2 = PoseStamped()
            ps2.header.frame_id = ps.header.frame_id
            ps2.header.stamp = Time().to_msg()  # 0 = derniere TF connue
            ps2.pose = ps.pose
            try:
                return self.tf_buffer.transform(ps2, cible, timeout=Duration(seconds=0.05))
            except Exception as e:
                self.get_logger().warn(f"TF {ps.header.frame_id} -> {cible} indisponible : {e}",
                                       throttle_duration_sec=5.0)
                return None

    def cb_image(self, msg):
        if self.K is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        coins, ids, _ = self.detect(gris)
        trouve = False
        if ids is not None:
            for c, i in zip(coins, ids.flatten()):
                if int(i) != self.tag_id:
                    continue
                ok, rvec, tvec = cv2.solvePnP(self.obj, c.reshape(4, 2), self.K, self.D,
                                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                trouve = True
                self.publier(msg, rvec, tvec)
                cv2.drawFrameAxes(img, self.K, self.D, rvec, tvec, 0.05)
            aruco.drawDetectedMarkers(img, coins, ids)
        now = self.get_clock().now()
        if (now - self.t_dbg).nanoseconds > 5e8:  # image de debug a 2 Hz
            self.t_dbg = now
            if not trouve:
                cv2.putText(img, f"tag {self.tag_id} non vu", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            dbg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            dbg.header = msg.header
            self.pub_dbg.publish(dbg)

    def publier(self, msg, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = quat_de_matrice(R)
        ps = PoseStamped()
        ps.header = msg.header
        if not ps.header.frame_id:
            ps.header.frame_id = 'camera_optical'
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in tvec.flatten())
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw

        # --- tag par rapport au robot (approche finale) ---
        pb = self.transforme(ps, self.base_frame)
        if pb is not None:
            self.pub_base.publish(pb)
            # normale du tag (son axe z, qui sort vers la camera) projetee au sol
            n = matrice_de_quat(pb.pose.orientation)[:, 2]
            angle = math.degrees(math.atan2(-n[1], -n[0]))  # 0 = le robot est bien en face
            self.get_logger().info(
                f"tag : devant {pb.pose.position.x:.3f} m, lateral {pb.pose.position.y:+.3f} m, "
                f"hauteur {pb.pose.position.z:.3f} m, angle {angle:+.1f} deg",
                throttle_duration_sec=0.5)

        # --- centre de la brique sur la carte (but Nav2) ---
        pm = self.transforme(ps, self.global_frame)
        if pm is not None:
            n = matrice_de_quat(pm.pose.orientation)[:, 2]
            nx, ny = n[0], n[1]
            norme = math.hypot(nx, ny)
            if norme < 0.3:
                return  # tag vu presque a plat : orientation peu fiable
            nx, ny = nx / norme, ny / norme
            cap = math.atan2(-ny, -nx)  # le robot doit regarder a l'oppose de la normale
            out = PoseStamped()
            out.header.stamp = pm.header.stamp
            out.header.frame_id = self.global_frame
            out.pose.position.x = pm.pose.position.x - self.demi_ep * nx
            out.pose.position.y = pm.pose.position.y - self.demi_ep * ny
            out.pose.position.z = 0.0
            out.pose.orientation.z = math.sin(cap / 2)
            out.pose.orientation.w = math.cos(cap / 2)
            self.pub_map.publish(out)


def main():
    rclpy.init()
    node = ArucoPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
