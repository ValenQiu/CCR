import cv2
import numpy as np
import cv2.aruco as aruco
import math

# 6X6
# Base ID:
base_id = 99
# Track ID:
track_id = 11

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

camera = cv2.VideoCapture(0)

relativervec, relativetvec = None, None


def cal_Points(ids, corners, rvec, tvec):
    # return three arrays: center, rvecs, tvecs
    # the first components in each arrays are the pose of the base marker
    # the second components in each arrays are the pose of the track marker
    if ids is not None:
        center = np.full([2, 2], None)
        rvecs = np.full(2, None)
        tvecs = [[0, 0, 0], [0, 0, 0]]
        for i in range(len(ids)):
            if ids[i] == base_id:
                corner_base = corners[i][0]
                center_x = (corner_base[0][0] + corner_base[2][0]) / 2
                center_y = (corner_base[0][1] + corner_base[2][1]) / 2
                center[0][0] = center_x
                center[0][1] = center_y
                rvecs[0] = rvec[i][0]
                tvecs[0] = tvec[i][0]
            elif ids[i] == track_id:
                corner_track = corners[i][0]
                center_x = (corner_track[0][0] + corner_track[2][0]) / 2
                center_y = (corner_track[0][1] + corner_track[2][1]) / 2
                center[1][0] = center_x
                center[1][1] = center_y
                rvecs[1] = rvec[i][0]
                tvecs[1] = tvec[i][0]
        if not np.any(center == None):
            center = np.int0(center)
        return center, rvecs, tvecs


def inversePerspective(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec


def relativePosion(rvec1, tvec1, rvec2, tvec2):
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))
    # Inverse the second marker
    invRvec, invTvec = inversePerspective(rvec2, tvec2)
    info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]
    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec


def arucoIdentify():
    global relativervec, relativetvec
    ret, frame = camera.read()
    color_image = cv2.resize(frame, (640, 480))
    # cv2.imshow("frame", frame)

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
        centers, rotationVec, transferVec = cal_Points(ids, corners, rvec, tvec)
        # print("centers\n", centers)
        # print("rotational Vectors\n", rotationVec)
        # print("transfer Vector", transferVec)
        if all(rotationalVector is not None for rotationalVector in rotationVec):
            # print(transferVec)
            relativervec, relativetvec = relativePosion(rotationVec[0], transferVec[0], rotationVec[1], transferVec[1])
            # print("relativervec\n", relativervec)
            # print("relativetvec\n", relativetvec)
            # print("type of relativervec: ", type(rotationVec))



while True:
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        break
    arucoIdentify()
    if relativervec is not None:
        print("relativervec\n", relativervec)
        print("relativetvec\n", relativetvec)
    # arucoIdentify()
