import sys, time
import cv2
import numpy as np

DICT = cv2.aruco.DICT_4X4_100            # couvre les ID 0 à 99 (voir le règlement)
SHOW = "--show" in sys.argv
CAM  = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 0

aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
params = cv2.aruco.DetectorParameters()
params.minMarkerPerimeterRate = 0.01
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

cap = cv2.VideoCapture(CAM, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
if not cap.isOpened():
    sys.exit(f"Impossible d'ouvrir /dev/video{CAM}")

n, t0 = 0, time.time()
while True:
    ok, frame = cap.read()
    if not ok:
        break

    corners, ids, _ = detector.detectMarkers(frame)
    if ids is not None:
        for tag_id, c in zip(ids.flatten(), corners):
            cx, cy = c[0].mean(axis=0)
            print(f"tag {tag_id}: x={cx:.0f} y={cy:.0f}")
            side = np.linalg.norm(c[0][0] - c[0][1])   # côté du tag en pixels
            print(f"tag {tag_id}: x={cx:.0f} y={cy:.0f} (côté {side:.0f}px)")

    n += 1
    if n % 30 == 0:
        print(f"--- {n / (time.time() - t0):.1f} fps")

    if SHOW:
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("aruco", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()