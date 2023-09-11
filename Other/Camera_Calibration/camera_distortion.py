import cv2
import numpy as np
import time

dist = np.array([-0.4473624, 0.27176798, -0.00062173, -0.0019694, -0.09317617])
newcameramtx = np.array([[1.07438647e+03, 0.00000000e+00, 5.86387042e+02],
                        [0.00000000e+00, 1.06347595e+03, 3.29518190e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
mtx = np.array([[1.07588078e+03, 0.00000000e+00, 5.87202624e+02],
                [0.00000000e+00, 1.07555501e+03, 3.19733956e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

camera = cv2.VideoCapture(0)

while True:
    ret, frame = camera.read()
    frame = cv2.resize(frame, (1080, 720))
    h1, w1 = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))
    frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow("Distortion", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
