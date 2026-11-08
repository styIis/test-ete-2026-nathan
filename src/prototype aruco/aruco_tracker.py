#!/usr/bin/env python3
import json
import subprocess

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

DICT = cv2.aruco.DICT_4X4_100  # couvre les ID 0 à 99


class ArucoTracker(Node):
    def __init__(self):
        super().__init__("aruco_tracker")
        self.declare_parameter("camera", 0)
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)
        self.declare_parameter("exposure", 150)  # 0 = laisser l'auto

        cam = self.get_parameter("camera").value
        exposure = self.get_parameter("exposure").value
        if exposure > 0:
            # même réglage que ce que tu as testé à la main
            subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/video{cam}",
                 f"--set-ctrl=auto_exposure=1,exposure_time_absolute={exposure},"
                 "exposure_dynamic_framerate=0"],
                check=False,
            )

        self.cap = cv2.VideoCapture(cam, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter("width").value)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter("height").value)
        if not self.cap.isOpened():
            raise RuntimeError(f"Impossible d'ouvrir /dev/video{cam}")

        aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
        params = cv2.aruco.DetectorParameters()
        params.minMarkerPerimeterRate = 0.01
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        self.pub_img = self.create_publisher(CompressedImage, "aruco/image/compressed", 1)
        self.pub_tags = self.create_publisher(String, "aruco/tags", 10)
        self.create_timer(0.001, self.step)  # cap.read() bloque jusqu'à la prochaine image

    def step(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Pas d'image")
            return
        stamp = self.get_clock().now().to_msg()

        corners, ids, _ = self.detector.detectMarkers(frame)
        tags = []
        if ids is not None:
            for tag_id, c in zip(ids.flatten(), corners):
                cx, cy = c[0].mean(axis=0)
                side = np.linalg.norm(c[0][0] - c[0][1])
                tags.append({"id": int(tag_id), "x": round(float(cx), 1),
                             "y": round(float(cy), 1), "side_px": round(float(side), 1)})
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            img = CompressedImage()
            img.header.stamp = stamp
            img.header.frame_id = "camera"
            img.format = "jpeg"
            img.data = jpg.tobytes()
            self.pub_img.publish(img)

        self.pub_tags.publish(String(data=json.dumps({"tags": tags})))


def main():
    rclpy.init()
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()