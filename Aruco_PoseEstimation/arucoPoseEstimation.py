import cv2
import cv2.aruco
import numpy as np
import cv2.aruco as aruco
import math

# Base ID:
base_id = 0
# Track ID:
track_id = 1

# Camera
fx = 1.07588078e+03
fy = 1.07555501e+03
cx = 5.87202624e+02
cy = 3.19733956e+02
k1 = -0.4473624
k2 = 0.27176798
p1 = -0.00062173
p2 = -0.0019694
k3 = -0.09317617
cameraMatrix = np.array([[fx, 0, cx],
                         [0, fy, cy],
                         [0, 0, 1]])
dist = np.array([k1, k2, p1, p2, k3])

camera = cv2.VideoCapture(1)


def DetectArucoPose():
    ret, frame = camera.read()
    color_image = cv2.resize(frame, (640, 480))
    h1, w1 = color_image.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (h1, w1), 0, (h1, w1))
    frame = cv2.undistort(color_image, cameraMatrix, dist, None, newCameraMatrix)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, dist)
        for i in range(rvec.shape[0]):
            tvecCopy = tvec[i, :, :] + [10, 0, 0]
            aruco.drawAxis(frame, cameraMatrix, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
            aruco.drawDetectedMarkers(frame, corners, ids)
            # print("rvec[", i, ",: , ：]: ", rvec[i, :, :])
    cv2.imshow("arucoDetector", frame)


while True:
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        break
    DetectArucoPose()