import cv2

# Choisis le dictionnaire correspondant à tes tags (voir remarque ci-dessous)
DICT = cv2.aruco.DICT_4X4_50

aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

cap = cv2.VideoCapture(0)
while True:
    ok, frame = cap.read()
    if not ok:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        for tag_id, c in zip(ids.flatten(), corners):
            cx, cy = c[0].mean(axis=0)          # centre = moyenne des 4 coins
            print(f"tag {tag_id}: x={cx:.1f} y={cy:.1f}")
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow("aruco", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()